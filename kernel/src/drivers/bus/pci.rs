use core::fmt::Display;

use acpi::{PciAddress, platform::PciConfigRegions, sdt::mcfg::Mcfg};
use alloc::{alloc::Global, vec::Vec};
use pci_types::{
    Bar, CommandRegister, ConfigRegionAccess, DeviceId, DeviceRevision, EndpointHeader, HeaderType,
    Interface, MAX_BARS, PciHeader, PciPciBridgeHeader, SubsystemId, SubsystemVendorId, VendorId,
    device_type::DeviceType,
};
use rmm::{Arch, PageFlags, PageMapper, PhysicalAddress, VirtualAddress};
use spin::Mutex;

use crate::{arch::CurrentRmmArch, drivers::acpi::ACPI_TABLES, init::memory::FRAME_ALLOCATOR};

pub struct PciAccess<'a>(&'a PciConfigRegions<Global>);

impl<'a> PciAccess<'a> {
    pub fn new(regions: &'a PciConfigRegions<Global>) -> Self {
        Self(regions)
    }

    pub fn mmio_address(&self, address: PciAddress, offset: u16) -> VirtualAddress {
        let (segment, bus, device, function) = (
            address.segment(),
            address.bus(),
            address.device(),
            address.function(),
        );

        let physical_address = self
            .0
            .physical_address(segment, bus, device, function)
            .expect("Invalid PCI address") as usize
            + offset as usize;

        let physical_address = PhysicalAddress::new(physical_address);
        let virtual_address = unsafe { CurrentRmmArch::phys_to_virt(physical_address) };

        let mut frame_allocator = FRAME_ALLOCATOR.lock();
        let mut mapper = unsafe {
            PageMapper::<CurrentRmmArch, _>::current(rmm::TableKind::Kernel, &mut *frame_allocator)
        };

        if let Some(flusher) = unsafe {
            mapper.map_phys(
                virtual_address,
                physical_address,
                PageFlags::<CurrentRmmArch>::new().write(true),
            )
        } {
            flusher.flush();
        }

        virtual_address
    }
}

impl ConfigRegionAccess for PciAccess<'_> {
    unsafe fn read(&self, address: PciAddress, offset: u16) -> u32 {
        let address = self.mmio_address(address, offset);
        unsafe { core::ptr::read_volatile(address.data() as *const u32) }
    }

    unsafe fn write(&self, address: PciAddress, offset: u16, value: u32) {
        let address = self.mmio_address(address, offset);
        unsafe { core::ptr::write_volatile(address.data() as *mut u32, value) };
    }
}

#[derive(Debug)]
pub struct PciDevice {
    pub address: PciAddress,
    pub vendor_id: VendorId,
    pub device_id: DeviceId,
    pub subsystem_vendor_id: SubsystemVendorId,
    pub subsystem_device_id: SubsystemId,
    pub interface: Interface,
    pub revision: DeviceRevision,
    pub device_type: DeviceType,
    pub bars: [Option<Bar>; MAX_BARS],
}

impl Display for PciDevice {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(
            f,
            "{}:{}.{}: {:?} [{:04x}:{:04x}] [{:04x}:{:04x}] (rev: {:02x})",
            self.address.bus(),
            self.address.device(),
            self.address.function(),
            self.device_type,
            self.vendor_id,
            self.device_id,
            self.subsystem_vendor_id,
            self.subsystem_device_id,
            self.revision,
        )
    }
}

pub struct PciResolver<'a> {
    access: PciAccess<'a>,
    devices: Vec<PciDevice>,
}

impl<'a> PciResolver<'a> {
    fn resolve(access: PciAccess<'a>) -> Vec<PciDevice> {
        let mut resolver = Self {
            access,
            devices: Vec::new(),
        };

        for region in resolver.access.0.regions.iter() {
            resolver.scan_segment(region.pci_segment_group);
        }

        resolver.devices
    }

    fn scan_segment(&mut self, segment: u16) {
        self.scan_bus(segment, 0);

        let address = PciAddress::new(segment, 0, 0, 0);
        if PciHeader::new(address).has_multiple_functions(&self.access) {
            (1..8).for_each(|i| self.scan_bus(segment, i));
        }
    }

    fn scan_bus(&mut self, segment: u16, bus: u8) {
        (0..32).for_each(|device| {
            let address = PciAddress::new(segment, bus, device, 0);
            self.scan_function(segment, bus, device, 0);

            let header = PciHeader::new(address);
            if header.has_multiple_functions(&self.access) {
                (1..8).for_each(|function| {
                    self.scan_function(segment, bus, device, function);
                });
            }
        });
    }

    fn scan_function(&mut self, segment: u16, bus: u8, device: u8, function: u8) {
        let address = PciAddress::new(segment, bus, device, function);
        let header = PciHeader::new(address);

        let (vendor_id, device_id) = header.id(&self.access);
        let (revision, class, sub_class, interface) = header.revision_and_class(&self.access);

        if vendor_id == 0xffff {
            return;
        }

        let endpoint_bars = |header: &EndpointHeader| {
            let mut bars = [None; 6];
            let mut skip_next = false;

            for (index, bar_slot) in bars.iter_mut().enumerate() {
                if skip_next {
                    skip_next = false;
                    continue;
                }
                let bar = header.bar(index as u8, &self.access);
                if let Some(Bar::Memory64 { .. }) = bar {
                    skip_next = true;
                }
                *bar_slot = bar;
            }

            bars
        };

        match header.header_type(&self.access) {
            HeaderType::Endpoint => {
                let mut endpoint_header = EndpointHeader::from_header(header, &self.access)
                    .expect("Invalid endpoint header");

                let (subsystem_vendor_id, subsystem_device_id) =
                    endpoint_header.subsystem(&self.access);

                let bars = endpoint_bars(&endpoint_header);
                let device_type = DeviceType::from((class, sub_class));

                endpoint_header.update_command(&self.access, |command| {
                    command
                        | CommandRegister::BUS_MASTER_ENABLE
                        | CommandRegister::IO_ENABLE
                        | CommandRegister::MEMORY_ENABLE
                });

                let device = PciDevice {
                    address,
                    vendor_id,
                    device_id,
                    subsystem_vendor_id,
                    subsystem_device_id,
                    interface,
                    device_type,
                    revision,
                    bars,
                };

                self.devices.push(device);
            }
            HeaderType::PciPciBridge => {
                let bridge_header = PciPciBridgeHeader::from_header(header, &self.access)
                    .expect("Invalid PCI-PCI bridge header");

                let start_bus = bridge_header.secondary_bus_number(&self.access);
                let end_bus = bridge_header.subordinate_bus_number(&self.access);
                (start_bus..=end_bus).for_each(|bus_id| self.scan_bus(segment, bus_id));
            }
            _ => {}
        }
    }
}

pub static PCI_DEVICES: Mutex<Option<Vec<PciDevice>>> = Mutex::new(None);

pub fn init() {
    if let Some(acpi_tables) = ACPI_TABLES.lock().as_mut() {
        if let Ok(pci_regions) = PciConfigRegions::new(acpi_tables) {
            let access = PciAccess::new(&pci_regions);
            let devices = PciResolver::resolve(access);
            devices.iter().for_each(|device| debug!("{}", device));
            *PCI_DEVICES.lock() = Some(devices);
        }
    }
}
