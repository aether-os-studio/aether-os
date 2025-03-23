#![no_std]
#![no_main]
#![allow(unsafe_op_in_unsafe_fn)]
#![feature(abi_x86_interrupt)]
#![feature(alloc_error_handler)]
#![feature(allocator_api)]
#![feature(naked_functions)]

use limine::mp::Cpu;
use pctable::idt::IDT;
use smp::{BSP_LAPIC_ID, CPUS};

extern crate alloc;

#[macro_use]
extern crate log;

mod acpi;
mod apic;
mod kdevice;
mod memory;
mod pctable;
mod smp;

pub fn init() {
    memory::init_heap();

    kdevice::log::init();

    info!("Aether OS kernel starting...");

    CPUS.write().load(*BSP_LAPIC_ID);
    IDT.load();
    CPUS.write().init_ap();

    apic::init();
}

unsafe extern "C" fn ap_entry(smp_info: &Cpu) -> ! {
    CPUS.write().load(smp_info.lapic_id);
    IDT.load();

    debug!("Application Processor {} started", smp_info.id);

    hcf()
}

#[panic_handler]
fn rust_panic(info: &core::panic::PanicInfo) -> ! {
    error!("{}", info);
    hcf();
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
