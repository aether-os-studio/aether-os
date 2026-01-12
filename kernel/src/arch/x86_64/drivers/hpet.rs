use core::time::Duration;

use acpi::HpetInfo;
use bit_field::BitField;
use rmm::{Arch, BuddyAllocator, PageFlags, PageMapper, PhysicalAddress};
use spin::Lazy;

use crate::{arch::CurrentRmmArch, drivers::acpi::ACPI_TABLES, init::memory::FRAME_ALLOCATOR};

pub static HPET: Lazy<Hpet> = Lazy::new(|| {
    if let Some(acpi_tables) = ACPI_TABLES.lock().as_mut() {
        let hpet_info = HpetInfo::new(acpi_tables).expect("Failed to get hpet info");
        let physical_address = PhysicalAddress::new(hpet_info.base_address);
        let virtual_address = unsafe { CurrentRmmArch::phys_to_virt(physical_address) };

        let mut frame_allocator = FRAME_ALLOCATOR.lock();
        let mut mapper = unsafe {
            PageMapper::<CurrentRmmArch, &mut BuddyAllocator<CurrentRmmArch>>::current(
                rmm::TableKind::Kernel,
                &mut *frame_allocator,
            )
        };

        unsafe {
            mapper.map_phys(
                virtual_address,
                physical_address,
                PageFlags::new().write(true),
            )
        };

        Hpet::new(virtual_address.data() as u64)
    } else {
        panic!("Failed to get acpi tables");
    }
});

pub struct Hpet {
    address: u64,
    fms_per_tick: u64,
}

impl Hpet {
    pub fn ticks(&self) -> u64 {
        let counter_addr = (self.address + 0xf0) as *const u64;
        unsafe { core::ptr::read_volatile(counter_addr) }
    }

    pub fn elapsed(&self) -> Duration {
        let ticks = self.ticks();
        Duration::from_nanos(ticks * self.fms_per_tick / 1_000_000)
    }

    pub fn estimate(&self, duration: Duration) -> u64 {
        let ticks = self.ticks();
        ticks + (duration.as_nanos() as u64 * 1_000_000 / self.fms_per_tick)
    }
}

impl Hpet {
    pub fn new(address: u64) -> Self {
        let general_ptr = address as *const u64;
        let general_info = unsafe { core::ptr::read_volatile(general_ptr) };

        let fms_per_tick = general_info.get_bits(32..64);
        let counter_addr = (address + 0xf0) as *const u64;
        unsafe { core::ptr::write_volatile(counter_addr as *mut u64, 0) };

        let hpet = Self {
            address,
            fms_per_tick,
        };

        unsafe {
            let enable_cnf_addr = (hpet.address + 0x10) as *mut u64;
            let old_cnf = core::ptr::read_volatile(enable_cnf_addr);
            core::ptr::write_volatile(enable_cnf_addr, old_cnf | 1);
        }

        hpet
    }
}
