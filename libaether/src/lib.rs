#![no_std]
#![no_main]
#![allow(unsafe_op_in_unsafe_fn)]

use asc::syscall;

pub unsafe fn put_string(str: &str) {
    syscall!(SYS_PUTSTRING, str.as_ptr() as usize, str.len());
}

pub unsafe fn fork() -> isize {
    syscall!(SYS_FORK)
}

pub unsafe fn vfork() -> isize {
    syscall!(SYS_VFORK)
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        unsafe { put_string("User panic!!!") };
    }
}
