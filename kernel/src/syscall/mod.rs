use crate::errno::Errno;

use crate::arch::nr::*;

pub type Result<T> = core::result::Result<T, Errno>;

pub fn handle_syscall(
    idx: usize,
    args: (usize, usize, usize, usize, usize, usize),
    regs_addr: usize,
) -> crate::syscall::Result<usize> {
    let mut res = Ok(0);

    match idx {
        SYS_OPEN => {}
        SYS_CLOSE => {}
        SYS_READ => {}
        SYS_WRITE => {}
        SYS_READV => {}
        SYS_WRITEV => {}
        // network?
        SYS_SOCKET => {}
        SYS_BIND => {}
        SYS_CONNECT => {}
        SYS_LISTEN => {}
        SYS_ACCEPT => {}
        SYS_SENDTO => {}
        SYS_RECVFROM => {}
        SYS_SENDMSG => {}
        SYS_RECVMSG => {}
        // Process management
        SYS_ARCH_PRCTL => {}
        SYS_FORK => {}
        SYS_VFORK => {}
        SYS_EXECVE => {}
        _ => {
            res = Err(Errno::ENOSYS);
        }
    }

    res
}
