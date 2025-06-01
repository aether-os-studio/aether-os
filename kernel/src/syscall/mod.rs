use mm::sys_brk;
use mm::sys_mmap;
use rmm::Arch;

mod mm;

use crate::errno::Errno;

use crate::arch::CurrentMMArch;
use crate::arch::nr::*;
use crate::fs::syscall::sys_write;
use crate::fs::syscall::{sys_close, sys_open, sys_read};
use crate::proc::exec::sys_execve;
use crate::proc::sched::get_current_context;
use crate::proc::signal::sys_rt_sigaction;
use crate::proc::signal::sys_rt_sigprocmask;
use crate::proc::signal::sys_sigaltstack;
use crate::proc::syscall::sys_fork;

pub fn check_user_overflows(addr: usize, size: usize) -> bool {
    if let Some(addr) = addr.checked_add(size) {
        if (addr + size) >= CurrentMMArch::PHYS_OFFSET {
            return true;
        }
        return false;
    }
    true
}

pub fn slice_from_user<T>(addr: *const T, size: usize) -> Option<&'static [T]> {
    let overflow = check_user_overflows(addr as usize, size);
    if overflow {
        return None;
    }

    Some(unsafe { core::slice::from_raw_parts(addr, size) })
}

pub fn slice_from_user_mut<T>(addr: *mut T, size: usize) -> Option<&'static mut [T]> {
    let overflow = check_user_overflows(addr as usize, size);
    if overflow {
        return None;
    }

    Some(unsafe { core::slice::from_raw_parts_mut(addr, size) })
}

pub type Result<T> = core::result::Result<T, Errno>;

pub fn handle_syscall(
    idx: usize,
    args: (usize, usize, usize, usize, usize, usize),
    regs_addr: usize,
) -> Result<usize> {
    let mut res = Ok(0);

    res = match idx {
        SYS_OPEN => sys_open(
            args.0 as *const i8,
            args.1 as core::ffi::c_int,
            args.2 as core::ffi::c_int,
        ),
        SYS_CLOSE => sys_close(args.0 as core::ffi::c_int),
        SYS_READ => {
            if let Some(buf) = slice_from_user_mut(args.1 as *mut u8, args.2) {
                return sys_read(args.0, buf);
            }
            Err(Errno::EINVAL)
        }
        SYS_WRITE => {
            if let Some(buf) = slice_from_user(args.1 as *const u8, args.2) {
                return sys_write(args.0, buf);
            }
            Err(Errno::EINVAL)
        }
        SYS_READV => Err(Errno::ENOSYS),
        SYS_WRITEV => Err(Errno::ENOSYS),
        // network
        SYS_SOCKET => Err(Errno::ENOSYS),
        SYS_BIND => Err(Errno::ENOSYS),
        SYS_CONNECT => Err(Errno::ENOSYS),
        SYS_LISTEN => Err(Errno::ENOSYS),
        SYS_ACCEPT => Err(Errno::ENOSYS),
        SYS_SENDTO => Err(Errno::ENOSYS),
        SYS_RECVFROM => Err(Errno::ENOSYS),
        SYS_SENDMSG => Err(Errno::ENOSYS),
        SYS_RECVMSG => Err(Errno::ENOSYS),
        // Process management
        SYS_FORK => sys_fork(regs_addr),
        SYS_VFORK => sys_fork(regs_addr),
        SYS_EXECVE => {
            sys_execve(
                args.0 as *const core::ffi::c_char,
                args.1 as *const *mut core::ffi::c_char,
                args.2 as *const *mut core::ffi::c_char,
            )?;

            Ok(0)
        }
        SYS_SET_TID_ADDRESS => Ok(0),
        SYS_RT_SIGACTION => sys_rt_sigaction(
            args.0,
            args.1 as *const SigAction,
            args.2 as *mut SigAction,
            args.3,
        ),
        SYS_RT_SIGPROCMASK => {
            sys_rt_sigprocmask(args.0, args.1 as *const u64, args.2 as *mut u64, args.3)
        }
        SYS_SIGALTSTACK => {
            sys_sigaltstack(args.0 as *const SignalStack, args.1 as *mut SignalStack)
        }
        SYS_GETPID => {
            if args.0 == usize::MAX
                && args.1 == usize::MAX
                && args.2 == usize::MAX
                && args.3 == usize::MAX
                && args.4 == usize::MAX
            {
                return Ok(1);
            }
            Ok(get_current_context().read().get_pid())
        }
        SYS_GETTID => Ok(get_current_context().read().get_pid()),
        // Memory management
        SYS_BRK => sys_brk(args.0),
        SYS_MMAP => sys_mmap(args.0, args.1, args.2, args.3, args.4, args.5),
        _ => Err(Errno::ENOSYS),
    };

    if let Ok(e) = res {
        Ok(e)
    } else {
        let err = res.expect_err("OOM");
        if err == Errno::ENOSYS {
            log::warn!("Syscall {} not implemented", idx);
        }
        Err(err)
    }
}
