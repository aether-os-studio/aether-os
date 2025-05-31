use rmm::Arch;

use crate::errno::Errno;

use crate::arch::CurrentMMArch;
use crate::arch::nr::*;
use crate::fs::syscall::sys_write;
use crate::fs::syscall::{sys_close, sys_open, sys_read};
use crate::proc::exec::sys_execve;

pub fn check_user_overflows(addr: usize, size: usize) -> bool {
    if let Some(addr) = addr.checked_add(size) {
        if addr >= CurrentMMArch::PHYS_OFFSET {
            return true;
        }
    }
    false
}

pub fn slice_from_user(addr: usize, size: usize) -> Option<&'static [u8]> {
    let overflow = check_user_overflows(addr, size);
    if overflow {
        return None;
    }

    Some(unsafe { core::slice::from_raw_parts(addr as *const u8, size) })
}

pub fn slice_from_user_mut(addr: usize, size: usize) -> Option<&'static mut [u8]> {
    let overflow = check_user_overflows(addr, size);
    if overflow {
        return None;
    }

    Some(unsafe { core::slice::from_raw_parts_mut(addr as *mut u8, size) })
}

pub type Result<T> = core::result::Result<T, Errno>;

pub fn handle_syscall(
    idx: usize,
    args: (usize, usize, usize, usize, usize, usize),
    regs_addr: usize,
) -> crate::syscall::Result<usize> {
    let mut res = Ok(0);

    res = match idx {
        SYS_OPEN => sys_open(
            args.0 as *const i8,
            args.1 as core::ffi::c_int,
            args.2 as core::ffi::c_int,
        ),
        SYS_CLOSE => sys_close(args.0 as core::ffi::c_int),
        SYS_READ => {
            if let Some(buf) = slice_from_user_mut(args.1, args.2) {
                return sys_read(args.0, buf);
            }
            Err(Errno::EINVAL)
        }
        SYS_WRITE => {
            if let Some(buf) = slice_from_user(args.1, args.2) {
                return sys_write(args.0, buf);
            }
            Err(Errno::EINVAL)
        }
        SYS_READV => Err(Errno::ENOSYS),
        SYS_WRITEV => Err(Errno::ENOSYS),
        // network?
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
        SYS_FORK => Err(Errno::ENOSYS),
        SYS_VFORK => Err(Errno::ENOSYS),
        SYS_EXECVE => {
            sys_execve(
                args.0 as *const core::ffi::c_char,
                args.1 as *const *mut core::ffi::c_char,
                args.2 as *const *mut core::ffi::c_char,
            )?;

            Ok(0)
        }
        _ => Err(Errno::ENOSYS),
    };

    res
}
