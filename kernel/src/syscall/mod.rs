use crate::errno::Errno;

use crate::arch::nr::*;
use crate::fs::syscall::sys_open;

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
        SYS_CLOSE => Err(Errno::ENOSYS),
        SYS_READ => Err(Errno::ENOSYS),
        SYS_WRITE => Err(Errno::ENOSYS),
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
        SYS_ARCH_PRCTL => Err(Errno::ENOSYS),
        SYS_FORK => Err(Errno::ENOSYS),
        SYS_VFORK => Err(Errno::ENOSYS),
        SYS_EXECVE => Err(Errno::ENOSYS),
        _ => Err(Errno::ENOSYS),
    };

    res
}
