use core::ptr::NonNull;

use crate::{
    arch::{CurrentRmmArch, CurrentTimeArch, rmm::page_flags, time::TimeArch},
    init::memory::{FRAME_ALLOCATOR, PAGE_SIZE, align_down, align_up},
};
use acpi::AcpiTables;
use limine::request::RsdpRequest;
use rmm::{Arch, PageMapper, PhysicalAddress};
use spin::{Lazy, Mutex};

#[derive(Clone)]
pub struct AcpiHandler;

#[allow(unused)]
impl acpi::Handler for AcpiHandler {
    unsafe fn map_physical_region<T>(
        &self,
        pa: usize,
        size: usize,
    ) -> acpi::PhysicalMapping<Self, T> {
        let physical_address = align_down(pa);
        let offset = pa - physical_address;
        let physical_address = PhysicalAddress::new(physical_address);
        let virtual_address = CurrentRmmArch::phys_to_virt(physical_address);
        let size = align_up(size);

        let mut frame_allocator = FRAME_ALLOCATOR.lock();
        let mut mapper =
            PageMapper::<CurrentRmmArch, _>::current(rmm::TableKind::Kernel, &mut *frame_allocator);

        for i in (0..size).step_by(PAGE_SIZE) {
            if let Some(flusher) = mapper.map_phys(
                virtual_address.add(i),
                physical_address.add(i),
                page_flags(virtual_address.add(i)),
            ) {
                flusher.flush();
            }
        }

        acpi::PhysicalMapping {
            physical_start: physical_address.add(offset).data(),
            virtual_start: NonNull::new_unchecked(virtual_address.add(offset).data() as *mut T),
            region_length: size,
            mapped_length: size,
            handler: self.clone(),
        }
    }

    fn unmap_physical_region<T>(region: &acpi::PhysicalMapping<Self, T>) {}

    fn read_u8(&self, address: usize) -> u8 {
        unsafe { core::ptr::read_volatile(address as *const u8) }
    }

    fn read_u16(&self, address: usize) -> u16 {
        unsafe { core::ptr::read_volatile(address as *const u16) }
    }

    fn read_u32(&self, address: usize) -> u32 {
        unsafe { core::ptr::read_volatile(address as *const u32) }
    }

    fn read_u64(&self, address: usize) -> u64 {
        unsafe { core::ptr::read_volatile(address as *const u64) }
    }

    fn write_u8(&self, address: usize, value: u8) {
        unsafe {
            core::ptr::write_volatile(address as *mut _, value);
        }
    }
    fn write_u16(&self, address: usize, value: u16) {
        unsafe {
            core::ptr::write_volatile(address as *mut _, value);
        }
    }
    fn write_u32(&self, address: usize, value: u32) {
        unsafe {
            core::ptr::write_volatile(address as *mut _, value);
        }
    }
    fn write_u64(&self, address: usize, value: u64) {
        unsafe {
            core::ptr::write_volatile(address as *mut _, value);
        }
    }

    fn read_io_u8(&self, port: u16) -> u8 {
        0
    }

    fn read_io_u16(&self, port: u16) -> u16 {
        0
    }

    fn read_io_u32(&self, port: u16) -> u32 {
        0
    }

    fn write_io_u8(&self, port: u16, value: u8) {}

    fn write_io_u16(&self, port: u16, value: u16) {}

    fn write_io_u32(&self, port: u16, value: u32) {}

    fn read_pci_u8(&self, address: acpi::PciAddress, offset: u16) -> u8 {
        0
    }

    fn read_pci_u16(&self, address: acpi::PciAddress, offset: u16) -> u16 {
        0
    }

    fn read_pci_u32(&self, address: acpi::PciAddress, offset: u16) -> u32 {
        0
    }

    fn write_pci_u8(&self, address: acpi::PciAddress, offset: u16, value: u8) {}

    fn write_pci_u16(&self, address: acpi::PciAddress, offset: u16, value: u16) {}

    fn write_pci_u32(&self, address: acpi::PciAddress, offset: u16, value: u32) {}

    fn nanos_since_boot(&self) -> u64 {
        CurrentTimeArch::nano_time()
    }

    fn stall(&self, microseconds: u64) {
        let nanoseconds = microseconds * 1000000;
        CurrentTimeArch::delay(nanoseconds);
    }

    fn sleep(&self, milliseconds: u64) {
        let nanoseconds = milliseconds * 1000;
        CurrentTimeArch::delay(nanoseconds);
    }
}

#[used]
#[unsafe(link_section = ".requests")]
static RSDP_REQUEST: RsdpRequest = RsdpRequest::new();

pub static ACPI_TABLES: Lazy<Mutex<Option<AcpiTables<AcpiHandler>>>> = Lazy::new(|| {
    let result = if let Some(rsdp_response) = RSDP_REQUEST.get_response() {
        unsafe { AcpiTables::from_rsdp(AcpiHandler, rsdp_response.address()) }.ok()
    } else {
        None
    };
    Mutex::new(result)
});
