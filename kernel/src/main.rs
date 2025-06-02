#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(internal_features)]
#![allow(stable_features)]
#![allow(unsafe_op_in_unsafe_fn)]
#![allow(unused_assignments)]
#![allow(unused_variables)]
#![allow(static_mut_refs)]
#![feature(abi_x86_interrupt)]
#![feature(allocator_api)]
#![feature(naked_functions)]
#![feature(core_intrinsics)]
#![feature(sync_unsafe_cell)]

extern crate alloc;

#[macro_use]
extern crate log;

mod arch;
mod drivers;
mod errno;
mod fs;
mod memory;
mod net;
mod proc;
mod serial;
mod syscall;
mod time;

use core::hint::spin_loop;
use core::sync::atomic::{AtomicBool, AtomicUsize};

use alloc::ffi::CString;
use limine::BaseRevision;
use limine::request::{RequestsEndMarker, RequestsStartMarker};
use proc::exec::sys_execve;

use crate::arch::{arch_disable_intr, arch_enable_intr};

/// Sets the base revision to the latest revision supported by the crate.
/// See specification for further info.
/// Be sure to mark all limine requests with #[used], otherwise they may be removed by the compiler.
#[used]
// The .requests section allows limine to find the requests faster and more safely.
#[unsafe(link_section = ".requests")]
static BASE_REVISION: BaseRevision = BaseRevision::new();

/// Define the stand and end markers for Limine requests.
#[used]
#[unsafe(link_section = ".requests_start_marker")]
static _START_MARKER: RequestsStartMarker = RequestsStartMarker::new();
#[used]
#[unsafe(link_section = ".requests_end_marker")]
static _END_MARKER: RequestsEndMarker = RequestsEndMarker::new();

pub static CPU_COUNT: AtomicUsize = AtomicUsize::new(0);
pub static ENABLE_SCHEDULER: AtomicBool = AtomicBool::new(false);

#[unsafe(no_mangle)]
unsafe extern "C" fn kmain() -> ! {
    // All limine requests must also be referenced in a called function, otherwise they may be
    // removed by the linker.
    assert!(BASE_REVISION.is_supported());

    arch_disable_intr();

    memory::init();
    memory::heap::init();

    arch::init();

    drivers::init();

    proc::init();

    drivers::init_after_proc();

    fs::init();

    ENABLE_SCHEDULER.store(true, core::sync::atomic::Ordering::SeqCst);

    hcf();
}

fn init() -> ! {
    info!("init thread is running");

    info!("Ready to run /bin/bash");

    sys_execve(
        CString::new("/bin/bash").unwrap().as_ptr(),
        core::ptr::null(),
        core::ptr::null(),
    )
    .expect("Failed to execute /bin/bash");

    error!("Failed to run /bin/bash");

    hcf()
}

#[panic_handler]
fn rust_panic(info: &core::panic::PanicInfo) -> ! {
    error!("{}", info);
    loop {
        spin_loop();
    }
}

fn hcf() -> ! {
    loop {
        arch_enable_intr();
        spin_loop();
    }
}
