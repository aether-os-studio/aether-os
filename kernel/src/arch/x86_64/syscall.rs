use x86_64::VirtAddr;
use x86_64::registers::model_specific::{Efer, EferFlags};
use x86_64::registers::model_specific::{LStar, SFMask, Star};
use x86_64::registers::rflags::RFlags;

use crate::arch::gdt::Selectors;

use super::nr::SYS_ARCH_PRCTL;
use super::proc::context::ContextArch;
use super::proc::syscall::sys_arch_prctl;

pub fn init() {
    SFMask::write(RFlags::INTERRUPT_FLAG);
    LStar::write(VirtAddr::from_ptr(syscall_exception as *const ()));

    let (code_selector, data_selector) = Selectors::get_kernel_segments();
    let (user_code_selector, user_data_selector) = Selectors::get_user_segments();

    Star::write(
        user_code_selector,
        user_data_selector,
        code_selector,
        data_selector,
    )
    .unwrap();

    unsafe {
        Efer::write(Efer::read() | EferFlags::SYSTEM_CALL_EXTENSIONS);
    }
}

fn syscall_handler(ptr: *mut ContextArch) -> *mut ContextArch {
    let regs = &mut unsafe { *ptr };

    let idx = regs.rax;
    let args = (regs.rdi, regs.rsi, regs.rdx, regs.r10, regs.r8, regs.r9);

    match idx {
        SYS_ARCH_PRCTL => regs.rax = sys_arch_prctl(args.0, args.1),
        _ => {
            let res = crate::syscall::handle_syscall(idx, args, ptr as usize);
            if let Ok(res) = res {
                regs.rax = res;
            } else {
                regs.rax = res.expect_err("OOM").to_posix_errno() as usize;
            }
        }
    }

    return ptr;
}

#[unsafe(naked)]
unsafe extern "C" fn syscall_exception() {
    core::arch::naked_asm!(
        crate::push_context!(),
        "mov rdi, rsp",
        "call {syscall_handler}",
        "mov rsp, rax",
        crate::pop_context!(),
        "sysretq",
        syscall_handler = sym syscall_handler,
    );
}
