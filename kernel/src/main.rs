#![no_std]
#![no_main]
#![allow(dead_code)]
#![feature(abi_x86_interrupt)]
#![feature(allocator_api)]
#![feature(int_roundings)]
#![feature(let_chains)]
#![feature(naked_functions)]
#![feature(sync_unsafe_cell)]

mod acpi;
mod allocator;
mod arch;
mod context;
mod cpu_set;
mod klog;
mod memory;
mod startup;

extern crate alloc;

#[macro_use]
extern crate bitflags;

use core::sync::atomic::AtomicUsize;

use arch::apic::local::the_local_apic;
use cpu_set::LogicalCpuId;
use limine::request::{RequestsEndMarker, RequestsStartMarker};
use limine::BaseRevision;

/// Sets the base revision to the latest revision supported by the crate.
/// See specification for further info.
/// Be sure to mark all limine requests with #[used], otherwise they may be removed by the compiler.
#[used]
// The .requests section allows limine to find the requests faster and more safely.
#[unsafe(link_section = ".requests")]
static BASE_REVISION: BaseRevision = BaseRevision::new();

/// Define the stand and end markers for Limine requests.
#[used]
#[link_section = ".requests_start_marker"]
static _START_MARKER: RequestsStartMarker = RequestsStartMarker::new();
#[used]
#[link_section = ".requests_end_marker"]
static _END_MARKER: RequestsEndMarker = RequestsEndMarker::new();

pub static CPU_COUNT: AtomicUsize = AtomicUsize::new(0);

pub fn cpu_count() -> usize {
    CPU_COUNT.load(core::sync::atomic::Ordering::SeqCst)
}

pub fn cpu_id() -> LogicalCpuId {
    LogicalCpuId::new(unsafe { the_local_apic() }.id())
}

#[panic_handler]
fn rust_panic(info: &core::panic::PanicInfo) -> ! {
    log::error!("{}", info);
    hcf();
}

fn hcf() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

// TODO: Use this macro on aarch64 too.

macro_rules! linker_offsets(
    ($($name:ident),*) => {
        $(
        #[inline]
        pub fn $name() -> usize {
            extern "C" {
                // TODO: UnsafeCell?
                static $name: u8;
            }
            unsafe { &$name as *const u8 as usize }
        }
        )*
    }
);

pub const KERNEL_HEAP_OFFSET: usize = 0xFFFFFFFF_A0000000;
pub const KERNEL_HEAP_SIZE: usize = 8 * 1024 * 1024;

mod kernel_executable_offsets {
    linker_offsets!(
        __text_start,
        __text_end,
        __rodata_start,
        __rodata_end,
        __data_start,
        __data_end,
        __bss_start,
        __bss_end,
        __usercopy_start,
        __usercopy_end
    );

    #[cfg(target_arch = "x86_64")]
    linker_offsets!(__altrelocs_start, __altrelocs_end);
}

pub fn start_kernel() -> ! {
    loop {
        log::info!("Start kernel is running");
    }

    // crate::hcf()
}
