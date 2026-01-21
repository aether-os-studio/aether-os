use core::mem::offset_of;

use x86_64::{
    VirtAddr,
    registers::{
        control::{Efer, EferFlags},
        model_specific::{LStar, SFMask, Star},
        rflags::RFlags,
    },
};

use crate::{
    arch::{gdt::Selectors, syscall::SyscallArch},
    task::Task,
};

pub struct X8664SyscallArch;

impl SyscallArch for X8664SyscallArch {
    unsafe fn copy_from_user(dst: usize, src: usize, len: usize) -> usize {
        unsafe { core::ptr::copy_nonoverlapping(src as *const u8, dst as *mut u8, len) };
        0
    }

    unsafe fn copy_to_user(dst: usize, src: usize, len: usize) -> usize {
        unsafe { core::ptr::copy_nonoverlapping(src as *const u8, dst as *mut u8, len) };
        0
    }
}

#[unsafe(naked)]
unsafe extern "C" fn x8664_syscall_handler() {
    core::arch::naked_asm!(
        "mov gs:{user_syscall_stack_offset}, rsp",
        "mov rsp, gs:{syscall_stack_offset}",
        "sub rsp, 0x30",
        crate::push_context!(),
        "mov rdi, rsp",
        "call {syscall_handler}",
        crate::pop_context!(),
        "add rsp, 0x28",
        "mov rsp, gs:{user_syscall_stack_offset}",
        "sysretq",
        syscall_handler = sym crate::syscall::syscall_handler,
        user_syscall_stack_offset = const offset_of!(Task, user_syscall_stack),
        syscall_stack_offset = const offset_of!(Task, syscall_stack_top),
    );
}

pub fn init() {
    SFMask::write(RFlags::INTERRUPT_FLAG);
    LStar::write(VirtAddr::from_ptr(x8664_syscall_handler as *const ()));

    let (kernel_code, kernel_data) = Selectors::get_kernel_segments();
    let (user_code, user_data) = Selectors::get_user_segments();
    Star::write(user_code, user_data, kernel_code, kernel_data).unwrap();

    unsafe {
        Efer::write(Efer::read() | EferFlags::SYSTEM_CALL_EXTENSIONS);
    }
}
