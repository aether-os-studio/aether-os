#![no_std]
#![no_main]
#![allow(unsafe_op_in_unsafe_fn)]

use core::arch::asm;

#[inline(always)]
pub unsafe fn syscall0(mut n: usize) -> usize {
    asm!(
        "syscall",
        inout("rax") n,
        out("rcx") _,
        out("r11") _,
        options(nostack),
    );
    n
}

#[inline(always)]
pub unsafe fn syscall1(mut n: usize, a1: usize) -> usize {
    asm!(
        "syscall",
        inout("rax") n,
        in("rdi") a1,
        out("rcx") _,
        out("r11") _,
        options(nostack),
    );
    n
}

#[inline(always)]
pub unsafe fn syscall2(mut n: usize, a1: usize, a2: usize) -> usize {
    asm!(
        "syscall",
        inout("rax") n,
        in("rdi") a1,
        in("rsi") a2,
        out("rcx") _,
        out("r11") _,
        options(nostack),
    );
    n
}

#[inline(always)]
pub unsafe fn syscall3(mut n: usize, a1: usize, a2: usize, a3: usize) -> usize {
    asm!(
        "syscall",
        inout("rax") n,
        in("rdi") a1,
        in("rsi") a2,
        in("rdx") a3,
        out("rcx") _,
        out("r11") _,
        options(nostack),
    );
    n
}

#[inline(always)]
pub unsafe fn syscall4(mut n: usize, a1: usize, a2: usize, a3: usize, a4: usize) -> usize {
    asm!(
        "syscall",
        inout("rax") n,
        in("rdi") a1,
        in("rsi") a2,
        in("rdx") a3,
        in("r10") a4,
        out("rcx") _,
        out("r11") _,
        options(nostack),
    );
    n
}

#[inline(always)]
pub unsafe fn syscall5(
    mut n: usize,
    a1: usize,
    a2: usize,
    a3: usize,
    a4: usize,
    a5: usize,
) -> usize {
    asm!(
        "syscall",
        inout("rax") n,
        in("rdi") a1,
        in("rsi") a2,
        in("rdx") a3,
        in("r10") a4,
        in("r8") a5,
        out("rcx") _,
        out("r11") _,
        options(nostack),
    );
    n
}

#[inline(always)]
pub unsafe fn syscall6(
    mut n: usize,
    a1: usize,
    a2: usize,
    a3: usize,
    a4: usize,
    a5: usize,
    a6: usize,
) -> usize {
    asm!(
        "syscall",
        inout("rax") n,
        in("rdi") a1,
        in("rsi") a2,
        in("rdx") a3,
        in("r10") a4,
        in("r8") a5,
        in("r9") a6,
        out("rcx") _,
        out("r11") _,
        options(nostack),
    );
    n
}

#[unsafe(no_mangle)]
extern "C" fn _start() {
    unsafe { syscall3(1, 1, "Hello world!!!".as_ptr() as usize, 14) };

    loop {
        core::hint::spin_loop();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
