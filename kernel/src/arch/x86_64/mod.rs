pub mod acpi;
pub mod apic;
pub mod gdt;
pub mod hpet;
pub mod interrupts;
pub mod mp;
pub mod proc;
pub mod rmm;
pub mod syscall;

mod sys;
pub mod nr {
    pub use super::sys::*;
}

pub use ::rmm::X8664Arch as CurrentMMArch;

use mp::{BSP_LAPIC_ID, CPUS};
use x86_64::registers::control::{Cr0, Cr0Flags, Cr4, Cr4Flags};

pub fn cpu_init() {
    let mut cr0 = Cr0::read();
    cr0.remove(Cr0Flags::EMULATE_COPROCESSOR);
    cr0.insert(Cr0Flags::MONITOR_COPROCESSOR);
    unsafe { Cr0::write(cr0) };

    let mut cr4 = Cr4::read();
    cr4.insert(Cr4Flags::OSFXSR);
    cr4.insert(Cr4Flags::OSXMMEXCPT_ENABLE);
    cr4.insert(Cr4Flags::FSGSBASE);
    unsafe { Cr4::write(cr4) };
}

pub fn init() {
    cpu_init();
    acpi::init();
    CPUS.write().get_mut(*BSP_LAPIC_ID).init();
    interrupts::init();
    hpet::init();
    apic::init();
    CPUS.write().init_ap();
    syscall::init();
}

pub fn arch_enable_intr() {
    x86_64::instructions::interrupts::enable();
}

pub fn arch_disable_intr() {
    x86_64::instructions::interrupts::disable();
}
