use x86_64::VirtAddr;
use x86_64::registers::model_specific::{Efer, EferFlags};
use x86_64::registers::model_specific::{LStar, SFMask, Star};
use x86_64::registers::rflags::RFlags;

use crate::arch::gdt::Selectors;
use crate::errno::Errno;
use crate::fs::syscall::sys_poll;
use crate::time::PosixTimeSpec;

use super::nr::{PollFd, SYS_ARCH_PRCTL, SYS_CLOCK_GETTIME, SYS_POLL};
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

pub fn sys_clock_gettime(which: usize, ptr: *mut PosixTimeSpec) -> usize {
    unsafe { (*ptr).tv_sec = x86_rtc::Rtc::new().get_unix_timestamp() as i64 };
    unsafe { (*ptr).tv_nsec = 0 };
    0
}

fn syscall_handler(regs: &mut ContextArch) -> &mut ContextArch {
    regs.rip = regs.rcx;
    regs.rflags = regs.r11;
    regs.rsp = unsafe { (regs as *mut ContextArch).add(1) } as usize;
    let (cs, ds) = Selectors::get_user_segments();
    regs.cs = cs.0 as usize;
    regs.ss = ds.0 as usize;

    let idx = regs.rax;
    let args = (regs.rdi, regs.rsi, regs.rdx, regs.r10, regs.r8, regs.r9);

    match idx {
        SYS_ARCH_PRCTL => regs.rax = sys_arch_prctl(args.0, args.1),
        SYS_POLL => {
            regs.rax = sys_poll(args.0 as *mut PollFd, args.1, args.2 as isize)
                .unwrap_or(Errno::EINVAL.to_posix_errno() as usize);
        }
        SYS_CLOCK_GETTIME => regs.rax = sys_clock_gettime(args.0, args.1 as *mut PosixTimeSpec),
        _ => {
            let res = crate::syscall::handle_syscall(idx, args, regs as *mut ContextArch as usize);
            if let Ok(res) = res {
                regs.rax = res;
            } else {
                regs.rax = res.expect_err("OOM").to_posix_errno() as usize;
            }
        }
    }

    return regs;
}

#[unsafe(naked)]
unsafe extern "C" fn syscall_exception() {
    core::arch::naked_asm!(
        "sub rsp, 0x28",
        crate::push_context!(),
        "mov rdi, rsp",
        "call {syscall_handler}",
        "mov rsp, rax",
        crate::pop_context!(),
        "add rsp, 0x28",
        "sysretq",
        syscall_handler = sym syscall_handler,
    );
}
