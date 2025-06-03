use rmm::Arch;

mod mm;
use mm::*;

use crate::errno::Errno;

use crate::arch::CurrentMMArch;
use crate::arch::nr::*;
use crate::fs::syscall::WeirdPselect6;
use crate::fs::syscall::*;
use crate::fs::vfs::iov::IoVec;
use crate::proc::PosixOldUtsName;
use crate::proc::exec::sys_execve;
use crate::proc::sched::*;
use crate::proc::signal::*;
use crate::proc::syscall::*;
use crate::time::PosixTimeSpec;

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
        SYS_READV => sys_readv(args.0, args.1 as *const IoVec, args.2),
        SYS_WRITEV => sys_writev(args.0, args.1 as *const IoVec, args.2),
        SYS_FCNTL => sys_fcntl(args.0, args.1, args.2),
        SYS_DUP => sys_dup(args.0),
        SYS_FACCESSAT => sys_faccessat(args.0, args.1, args.2),
        SYS_FACCESSAT2 => sys_faccessat2(args.0, args.1, args.2, args.3),
        SYS_GETCWD => sys_getcwd(args.0, args.1),
        SYS_CHDIR => sys_chdir(args.0),
        SYS_FSTAT => sys_fstat(args.0, args.1),
        SYS_STAT => sys_stat(args.0 as *const core::ffi::c_char, args.1),
        SYS_LSEEK => sys_lseek(args.0, args.1, args.2),
        SYS_IOCTL => sys_ioctl(args.0, args.1, args.2),
        SYS_SELECT => sys_select(
            args.0,
            args.1,
            args.2,
            args.3,
            args.4 as *const PosixTimeSpec,
        ),
        SYS_PSELECT6 => sys_pselect6(
            args.0,
            args.1,
            args.2,
            args.3,
            args.4 as *const PosixTimeSpec,
            args.5 as *const WeirdPselect6,
        ),
        SYS_PIPE => {
            let pipefd = slice_from_user_mut(args.0 as *mut i32, 2).ok_or(Errno::EFAULT)?;
            sys_pipe(pipefd)
        }
        SYS_READLINK => {
            if let Some(buf) = slice_from_user_mut(args.1 as *mut u8, args.2) {
                return sys_readlink(args.0 as *const core::ffi::c_char, buf);
            }
            Err(Errno::EFAULT)
        }
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
            if check_user_overflows(args.1, args.3) || check_user_overflows(args.2, args.3) {
                return Err(Errno::EFAULT);
            }
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
        SYS_GETPPID => Ok(get_current_context().read().ppid),
        SYS_GETTID => Ok(get_current_context().read().get_pid()),
        SYS_EXIT => sys_exit(args.0),
        SYS_EXIT_GROUP => sys_exit(args.0),
        SYS_WAIT4 => sys_wait4(args.0, args.1, args.2, args.3),
        SYS_GETUID => Ok(0),
        SYS_GETGID => Ok(0),
        SYS_GETEUID => Ok(0),
        SYS_GETEGID => Ok(0),
        SYS_GETPGID => Ok(0),
        SYS_SETUID => Ok(0),
        SYS_SETGID => Ok(0),
        SYS_SETPGID => Ok(0),
        SYS_UNAME => sys_uname(args.0 as *mut PosixOldUtsName),
        SYS_GETRLIMIT => sys_getrlimit(args.0, args.1),
        SYS_PRLIMIT64 => sys_prlimit64(args.0, args.1, args.2),
        // Memory management
        SYS_BRK => sys_brk(args.0),
        SYS_MMAP => sys_mmap(args.0, args.1, args.2, args.3, args.4, args.5),
        SYS_MUNMAP => sys_munmap(args.0, args.1),
        SYS_MPROTECT => Ok(0),
        // Others
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
