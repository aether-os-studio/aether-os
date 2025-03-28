use alloc::sync::Arc;
use asc::nr::*;
use error::SystemError;
use num_traits::FromPrimitive;
use x86_64::VirtAddr;
use x86_64::registers::model_specific::{Efer, EferFlags};
use x86_64::registers::model_specific::{LStar, SFMask, Star};
use x86_64::registers::rflags::RFlags;
use x86_64::structures::tss::TaskStateSegment;

use crate::apic::LAPIC;
use crate::context::context::Context;
use crate::context::scheduler::SCHEDULER;
use crate::context::{get_current_process, get_current_process_id, get_current_thread};
use crate::fs::vfs::fcntl::FcntlCommand;
use crate::pctable::gdt::Selectors;
use crate::pctable::idt::InterruptIndex;
use crate::smp::CPUS;

pub mod fs;

pub fn init() {
    SFMask::write(RFlags::INTERRUPT_FLAG);
    LStar::write(VirtAddr::from_ptr(syscall_handler as *const ()));

    let (kernel_code, kernel_data) = Selectors::get_kernel_segments();
    let (user_code, user_data) = Selectors::get_user_segments();
    Star::write(user_code, user_data, kernel_code, kernel_data).unwrap();

    unsafe {
        Efer::write(Efer::read() | EferFlags::SYSTEM_CALL_EXTENSIONS);
    }
}

#[naked]
extern "C" fn syscall_handler() {
    unsafe {
        core::arch::naked_asm!(
            "sub rsp, 0x28",
            crate::push_context!(),
            "mov rdi, rax",
            "mov rsi, rsp",
            "call {syscall_matcher}",
            "mov rsp, rax",
            crate::pop_context!(),
            "add rsp, 0x28",
            "sysretq",
            syscall_matcher = sym syscall_matcher,
        );
    }
}

pub const ARCH_SET_GS: usize = 0x1001;
pub const ARCH_SET_FS: usize = 0x1002;
pub const ARCH_GET_FS: usize = 0x1003;
pub const ARCH_GET_GS: usize = 0x1004;

#[allow(unused_variables)]
pub extern "C" fn syscall_matcher(syscall_index: usize, context: &mut Context) -> VirtAddr {
    context.rip = context.rcx;
    context.rflags = context.r11;
    context.rsp = (context as *mut Context).wrapping_add(1) as usize;
    let (code, data) = Selectors::get_user_segments();
    context.cs = code.0 as usize;
    context.ss = data.0 as usize;

    let arg1 = context.rdi;
    let arg2 = context.rsi;
    let arg3 = context.rdx;
    let arg4 = context.r10;
    let arg5 = context.r8;
    let arg6 = context.r9;

    // debug!(
    //     "{}: {:#x}, {:#x}, {:#x}, {:#x}, {:#x}, {:#x}",
    //     syscall_index, arg1, arg2, arg3, arg4, arg5, arg6
    // );

    let result = match syscall_index {
        FORK => do_fork(context, false),
        VFORK => do_fork(context, true),
        EXIT => sys_exit(arg1),
        EXIT_GROUP => sys_exit(0),
        IOPL => {
            let allowed = arg1 >= 3;

            let offset = if allowed {
                u16::try_from(size_of::<TaskStateSegment>()).unwrap()
            } else {
                0xFFFF
            };

            let mut cpus = CPUS.write();
            cpus.get_mut(unsafe { LAPIC.lock().id() })
                .set_iomap_base(offset);

            0
        }
        BRK => sys_brk(arg1),
        MMAP => 0,
        MUNMAP => 0,
        OPEN => {
            let buf = unsafe { core::slice::from_raw_parts(arg1 as *const u8, arg2) };
            let str = str::from_utf8(buf).unwrap();

            fs::sys_open(str)
        }
        READ => {
            let buf = unsafe { core::slice::from_raw_parts_mut(arg2 as *mut u8, arg3) };

            fs::sys_read(arg1, buf)
        }
        WRITE => {
            let buf = unsafe { core::slice::from_raw_parts(arg2 as *const u8, arg3) };

            fs::sys_write(arg1, buf)
        }
        CLOSE => {
            let fd = arg1;
            fs::sys_close(fd)
        }
        FCNTL => {
            let fd = arg1;
            let cmd = <FcntlCommand as FromPrimitive>::from_usize(arg2);
            let arg = arg3;
            if let Some(cmd) = cmd {
                fs::sys_fcntl(fd, cmd, arg);
            }

            SystemError::EINVAL.to_posix_errno() as isize
        }
        GETCWD => {
            let buf = arg1 as *mut u8;
            let size = arg2;

            let buf = unsafe { core::slice::from_raw_parts_mut(buf, size) };

            fs::sys_getcwd(buf)
        }
        ARCH_PRCTL => match arg1 {
            ARCH_GET_FS => get_current_thread().read().context.fsbase as isize,
            ARCH_GET_GS => get_current_thread().read().context.gsbase as isize,
            ARCH_SET_FS => {
                get_current_thread().write().context.fsbase = arg2;
                context.fsbase = arg2;
                0
            }
            ARCH_SET_GS => {
                get_current_thread().write().context.gsbase = arg2;
                context.gsbase = arg2;
                0
            }
            _ => SystemError::EINVAL.to_posix_errno() as isize,
        },
        GETPID => get_current_process_id().0 as isize,
        SET_TID_ADDRESS => {
            get_current_thread().write().clear_child_tid = arg1;
            get_current_process_id().0 as isize
        }
        POLL => 0,
        RT_SIGACTION => 0,
        RT_SIGPROCMASK => 0,
        _ => SystemError::ENOSYS.to_posix_errno() as isize,
    };

    context.rax = result as usize;

    context.address()
}

pub fn do_fork(context: &mut Context, vfork: bool) -> isize {
    let current = SCHEDULER.lock().current();
    if let Some(current) = current.upgrade() {
        return current
            .read()
            .process
            .upgrade()
            .unwrap()
            .read()
            .do_fork(context, vfork);
    }

    0
}

pub fn sys_yield() -> isize {
    x86_64::instructions::interrupts::enable_and_hlt();

    unsafe {
        core::arch::asm!("int {}", const InterruptIndex::Timer as u8);
    }

    0
}

pub fn sys_brk(addr: usize) -> isize {
    let mut new_brk = (addr + 4095) & !0xFFFusize;

    if addr == 0 {
        return get_current_process().read().brk_start as isize;
    }
    if new_brk < get_current_process().read().brk_end {
        return 0;
    }

    new_brk = crate::memory::do_brk(
        get_current_process().read().brk_end,
        new_brk - get_current_process().read().brk_end,
    );

    get_current_process().write().brk_end = new_brk;

    return new_brk as isize;
}

pub fn sys_exit(_code: usize) -> isize {
    let task = SCHEDULER.lock().current();

    if let Some(task) = task.upgrade() {
        let mut scheduler = SCHEDULER.lock();
        task.read().process.upgrade().unwrap().read().exit();
        scheduler.remove(Arc::downgrade(&task));
    }

    sys_yield();

    SystemError::ESRCH.to_posix_errno() as isize
}
