pub mod acpi;
pub mod apic;
pub mod gdt;
pub mod hpet;
pub mod interrupts;
pub mod mp;
pub mod proc;
pub mod rmm;

pub use ::rmm::X8664Arch as CurrentMMArch;
use mp::{BSP_LAPIC_ID, CPUS};

pub fn init() {
    acpi::init();
    CPUS.write().get_mut(*BSP_LAPIC_ID).init();
    interrupts::init();
    hpet::init();
    apic::init();
    CPUS.write().init_ap();
}

pub fn arch_enable_intr() {
    x86_64::instructions::interrupts::enable();
}

pub fn arch_disable_intr() {
    x86_64::instructions::interrupts::disable();
}
