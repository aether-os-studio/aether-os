mod boot;
pub mod cache;
pub mod drivers;
pub mod gdt;
pub mod irq;
pub mod rmm;
pub mod smp;
pub mod time;

use crate::arch::x86_64::irq::IDT;

pub use self::cache::X8664CacheArch as CurrentCacheArch;
pub use self::irq::X8664IrqArch as CurrentIrqArch;
pub use self::time::X8664TimeArch as CurrentTimeArch;
pub use ::rmm::X8664Arch as CurrentRmmArch;
use x86_64::registers::control::{Cr0, Cr0Flags, Cr4, Cr4Flags};

pub fn init_sse() {
    let mut cr0 = Cr0::read();
    cr0.remove(Cr0Flags::EMULATE_COPROCESSOR);
    cr0.insert(Cr0Flags::MONITOR_COPROCESSOR);
    unsafe { Cr0::write(cr0) };

    let mut cr4 = Cr4::read();
    cr4.insert(Cr4Flags::OSFXSR);
    cr4.insert(Cr4Flags::OSXMMEXCPT_ENABLE);
    unsafe { Cr4::write(cr4) };
}

pub fn early_init() {
    init_sse();
    crate::smp::init();
    crate::arch::x86_64::irq::init();
    crate::arch::x86_64::drivers::apic::init();
}
