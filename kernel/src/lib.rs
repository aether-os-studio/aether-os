#![no_std]
#![no_main]
#![allow(unsafe_op_in_unsafe_fn)]
#![feature(abi_x86_interrupt)]
#![feature(alloc_error_handler)]
#![feature(allocator_api)]
#![feature(naked_functions)]

use core::sync::atomic::Ordering;

use apic::{APIC_INIT, LAPIC, LAPIC_TIMER_INITIAL};
use context::scheduler::SCHEDULER_INIT;
use limine::mp::Cpu;
use pctable::idt::IDT;
use smp::{BSP_LAPIC_ID, CPUS};
use x86_64::registers::control::{Cr0, Cr0Flags, Cr4, Cr4Flags};

extern crate alloc;

#[macro_use]
extern crate log;

pub mod acpi;
pub mod apic;
pub mod context;
pub mod kdevice;
pub mod memory;
pub mod pctable;
pub mod smp;
pub mod syscall;

mod unwind;

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

pub fn init() {
    init_sse();

    memory::init_heap();

    kdevice::log::init();

    info!("Aether OS kernel starting...");

    CPUS.write().load(*BSP_LAPIC_ID);
    IDT.load();
    CPUS.write().init_ap();

    apic::init();

    syscall::init();

    context::init();
}

unsafe extern "C" fn ap_entry(smp_info: &Cpu) -> ! {
    init_sse();

    CPUS.write().load(smp_info.lapic_id);
    IDT.load();

    while !APIC_INIT.load(Ordering::SeqCst) {
        core::hint::spin_loop()
    }
    LAPIC.lock().enable();

    let timer_initial = LAPIC_TIMER_INITIAL.load(Ordering::Relaxed);
    LAPIC.lock().set_timer_initial(timer_initial);

    syscall::init();

    while !SCHEDULER_INIT.load(Ordering::SeqCst) {
        core::hint::spin_loop()
    }

    debug!("Application Processor {} started", smp_info.id);

    hcf()
}

pub fn hcf() -> ! {
    loop {
        unsafe {
            #[cfg(target_arch = "x86_64")]
            {
                core::arch::asm!("sti");
                core::arch::asm!("hlt");
            }
            #[cfg(any(target_arch = "aarch64", target_arch = "riscv64"))]
            core::arch::asm!("wfi");
            #[cfg(target_arch = "loongarch64")]
            core::arch::asm!("idle 0");
        }
    }
}
