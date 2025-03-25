#![no_std]
#![no_main]
#![allow(unsafe_op_in_unsafe_fn)]

use core::hint::spin_loop;

#[unsafe(no_mangle)]
unsafe extern "C" fn _start() -> ! {
    libaether::put_string("all: Hello world!!!");

    let pid = libaether::vfork();
    if pid == 0 {
        libaether::put_string("child: Hello world!!!");
    } else {
        libaether::put_string("parent: Hello world!!!");
    }

    loop {
        spin_loop();
    }
}
