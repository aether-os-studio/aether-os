pub mod acpi;
pub mod apic;
pub mod gdt;
pub mod hpet;
pub mod interrupts;
pub mod proc;
pub mod rmm;

pub use ::rmm::X8664Arch as CurrentMMArch;

pub fn init() {
    acpi::init();
    gdt::init();
    interrupts::init();
    hpet::init();
    apic::init();
}

pub fn arch_enable_intr() {
    x86_64::instructions::interrupts::enable();
}

pub fn arch_disable_intr() {
    x86_64::instructions::interrupts::disable();
}
