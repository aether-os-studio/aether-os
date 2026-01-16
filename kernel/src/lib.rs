#![no_std]
#![no_main]
#![allow(clippy::field_reassign_with_default)]
#![allow(clippy::missing_safety_doc)]
#![allow(clippy::new_ret_no_self)]
#![allow(clippy::too_many_arguments)]
#![allow(unsafe_op_in_unsafe_fn)]
#![feature(abi_x86_interrupt)]
#![feature(allocator_api)]
#![feature(ptr_as_ref_unchecked)]
#![feature(sync_unsafe_cell)]

#[macro_use]
extern crate alloc;
#[macro_use]
extern crate log;

use limine::{
    BaseRevision,
    request::{RequestsEndMarker, RequestsStartMarker, StackSizeRequest},
};

#[used]
#[unsafe(link_section = ".requests")]
static BASE_REVISION: BaseRevision = BaseRevision::with_revision(3);

#[used]
#[unsafe(link_section = ".requests")]
static STACK_SIZE_REQUEST: StackSizeRequest =
    StackSizeRequest::new().with_size(consts::STACK_SIZE as u64);

#[used]
#[unsafe(link_section = ".requests_start_marker")]
static _START_MARKER: RequestsStartMarker = RequestsStartMarker::new();
#[used]
#[unsafe(link_section = ".requests_end_marker")]
static _END_MARKER: RequestsEndMarker = RequestsEndMarker::new();

pub mod arch;
pub mod consts;
pub mod drivers;
pub mod fs;
pub mod init;
pub mod memory;
pub mod smp;
pub mod task;
pub mod utils;

use core::{hint::spin_loop, panic::PanicInfo};

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("{}", info);
    loop {
        spin_loop();
    }
}

use crate::arch::{CurrentIrqArch, irq::IrqArch};

macro_rules! linker_offsets(
    ($($name:ident),*) => {
        $(
        #[inline]
        pub fn $name() -> usize {
            unsafe extern "C" {
                static $name: u8;
            }
            (&raw const $name) as usize
        }
        )*
    }
);
mod kernel_executable_offsets {
    linker_offsets!(__start, __end);
    linker_offsets!(__text_start, __text_end, __rodata_start, __rodata_end);
}

#[unsafe(no_mangle)]
extern "C" fn kmain() -> ! {
    CurrentIrqArch::disable_global_irq();

    memory::heap::init();

    #[cfg(not(target_arch = "x86_64"))]
    unsafe {
        drivers::dtb::init()
    };

    #[cfg(target_arch = "aarch64")]
    drivers::pl011::init();

    drivers::framebuffer::init();

    drivers::logger::init();

    arch::early_init();

    task::init();

    info!("Kernel initialized");

    loop {
        CurrentIrqArch::enable_global_irq();
        spin_loop();
    }
}

extern "C" fn initial_kernel_thread() -> ! {
    info!("Initial kernel thread is running");

    drivers::bus::pci::init();

    drivers::usb::init();

    drivers::storage::init();

    loop {
        spin_loop();
    }
}
