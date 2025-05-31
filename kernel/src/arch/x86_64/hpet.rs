use acpi::HpetInfo;
use core::{cell::UnsafeCell, ptr};
use rmm::{Arch, PageFlags, PhysicalAddress};

use crate::arch::CurrentMMArch;
use crate::memory::KERNEL_PAGE_TABLE;
use crate::{arch::acpi::ACPI, serial_println};

pub static HPET: Hpet = Hpet::uninit();

pub fn init() {
    let acpi = ACPI.get().unwrap();
    let acpi_tables = acpi.lock();
    let hpet = HpetInfo::new(&*acpi_tables).unwrap();
    let physical_address = PhysicalAddress::new(hpet.base_address as usize);
    let virtual_address = unsafe { CurrentMMArch::phys_to_virt(physical_address) };
    unsafe {
        let result = KERNEL_PAGE_TABLE.lock().map_phys(
            virtual_address,
            physical_address,
            PageFlags::new().write(true),
        );

        if let Some(flusher) = result {
            flusher.flush();
        }
    }

    HPET.init(virtual_address.data() as u64);
    HPET.enable_counter();

    serial_println!("HPET clock speed: {} femto seconds", HPET.clock_speed());
    serial_println!("HPET timers: {} available", HPET.timers_count());
}

pub struct Hpet {
    base_addr: UnsafeCell<u64>,
}

impl Hpet {
    #[inline]
    pub const fn uninit() -> Self {
        Hpet {
            base_addr: UnsafeCell::new(0),
        }
    }

    pub fn init(&self, base_addr: u64) {
        unsafe {
            self.base_addr.get().write(base_addr);
        }
    }

    pub fn clock_speed(&self) -> u32 {
        unsafe {
            let base_addr = *self.base_addr.get();
            let value = ptr::read_volatile(base_addr as *const u64);
            (value >> 32) as u32
        }
    }

    pub fn timers_count(&self) -> u32 {
        unsafe {
            let base_addr = *self.base_addr.get();
            let value = ptr::read_volatile(base_addr as *const u64);
            (((value >> 8) & 0b11111) + 1) as u32
        }
    }

    pub fn enable_counter(&self) {
        unsafe {
            let configuration_addr = *self.base_addr.get() + 0x10;
            let old = ptr::read_volatile(configuration_addr as *const u64);
            ptr::write_volatile(configuration_addr as *mut u64, old | 1);
        }
    }

    pub fn get_counter(&self) -> u64 {
        unsafe {
            let counter_l_addr = *self.base_addr.get() + 0xf0;
            let counter_h_addr = *self.base_addr.get() + 0xf4;
            loop {
                let high1 = ptr::read_volatile(counter_h_addr as *const u32);
                let low = ptr::read_volatile(counter_l_addr as *const u32);
                let high2 = ptr::read_volatile(counter_h_addr as *const u32);
                if high1 == high2 {
                    return (high1 as u64) << 32 | low as u64;
                }
            }
        }
    }

    #[inline]
    pub fn get_time_elapsed(&self) -> u64 {
        self.get_counter() * (self.clock_speed() as u64 / 1_000_000)
    }
}

unsafe impl Sync for Hpet {}
