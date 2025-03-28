use alloc::string::ToString;
use crossbeam_queue::ArrayQueue;
use error::SystemError;
use goto::gpoint;
use spin::Lazy;

use crate::{
    context::get_current_process,
    fs::vfs::{
        fcntl::{FD_CLOEXEC, FcntlCommand},
        get_inode_by_path,
        inode::FileMode,
    },
};

pub fn sys_open(path: &str) -> isize {
    let inode_res = get_inode_by_path(path.to_string());

    if let Err(err) = inode_res.clone() {
        return err.to_posix_errno() as isize;
    }

    let inode = inode_res.unwrap();
    let next_fd = get_current_process()
        .read()
        .next_fd
        .fetch_add(1, core::sync::atomic::Ordering::Relaxed);

    get_current_process()
        .write()
        .files
        .insert(next_fd, (inode.clone(), FileMode::O_RDWR, 0));

    return 0;
}

const KEYCODE_QUEUE_SIZE: usize = 128;

pub static KEYCODE_QUEUE: Lazy<ArrayQueue<u8>> = Lazy::new(|| ArrayQueue::new(KEYCODE_QUEUE_SIZE));

pub fn sys_read(fd: usize, data: &mut [u8]) -> isize {
    match fd {
        0 => {
            let mut write = 0;

            gpoint!('try_read:
            while let Some(byte) = KEYCODE_QUEUE.pop() {
                if write >= data.len() {
                    break;
                }
                data[write] = byte as u8;
                write += 1;
            }
            if data.len() > write {
                while let None = KEYCODE_QUEUE.pop() {
                    crate::syscall::sys_yield();
                }
                break 'try_read;
            });
            write as isize
        }
        1 | 2 => 0,
        _ => {
            if let Some((node, _, offset)) = get_current_process().read().files.get(&fd) {
                let result = node.read().read_at(*offset, data);
                if let Err(err) = result {
                    return err.to_posix_errno() as isize;
                }
                result.unwrap() as isize
            } else {
                return SystemError::EBADFD.to_posix_errno() as isize;
            }
        }
    }
}

pub fn sys_write(fd: usize, data: &[u8]) -> isize {
    match fd {
        0 => 0,
        1 => {
            let str = unsafe { str::from_utf8_unchecked(data) };
            crate::serial_print!("{}", str);
            crate::print!("{}", str);
            str.len() as isize
        }
        2 => {
            let str = unsafe { str::from_utf8_unchecked(data) };
            crate::serial_print!("\x1b[31m{}\x1b[0m", str);
            crate::print!("\x1b[31m{}\x1b[0m", str);
            str.len() as isize
        }
        _ => {
            if let Some((node, _, offset)) = get_current_process().read().files.get(&fd) {
                let result = node.read().write_at(*offset, data);
                if let Err(err) = result {
                    return err.to_posix_errno() as isize;
                }
                result.unwrap() as isize
            } else {
                return SystemError::EBADFD.to_posix_errno() as isize;
            }
        }
    }
}

pub fn sys_close(fd: usize) -> isize {
    if let Some(_) = get_current_process().write().files.remove(&fd) {
        return 0;
    }
    SystemError::EBADFD.to_posix_errno() as isize
}

pub fn sys_fcntl(fd: usize, cmd: FcntlCommand, arg: usize) -> isize {
    match cmd {
        FcntlCommand::DupFd => {
            if let Some((inode, mode, offset)) = get_current_process().write().files.get_mut(&fd) {
                let next_fd = get_current_process()
                    .read()
                    .next_fd
                    .fetch_add(1, core::sync::atomic::Ordering::Relaxed);

                get_current_process()
                    .write()
                    .files
                    .insert(next_fd, (inode.clone(), mode.clone(), *offset));
            }

            return SystemError::EBADF.to_posix_errno() as isize;
        }
        FcntlCommand::DupFdCloexec => {
            if let Some((inode, mode, offset)) = get_current_process().write().files.get_mut(&fd) {
                let next_fd = get_current_process()
                    .read()
                    .next_fd
                    .fetch_add(1, core::sync::atomic::Ordering::Relaxed);

                mode.insert(FileMode::O_CLOEXEC);

                get_current_process()
                    .write()
                    .files
                    .insert(next_fd, (inode.clone(), mode.clone(), *offset));
            }

            return SystemError::EBADF.to_posix_errno() as isize;
        }
        FcntlCommand::GetFd => {
            if let Some((_inode, mode, _offset)) = get_current_process().write().files.get_mut(&fd)
            {
                if mode.contains(FileMode::O_CLOEXEC) {
                    return FD_CLOEXEC as isize;
                } else {
                    return 0;
                }
            }

            return SystemError::EBADF.to_posix_errno() as isize;
        }
        FcntlCommand::GetFlags => {
            if let Some((_inode, mode, _offset)) = get_current_process().read().files.get(&fd) {
                return mode.bits() as isize;
            }

            return SystemError::EBADF.to_posix_errno() as isize;
        }
        FcntlCommand::SetFlags => {
            if let Some((_inode, mode, _offset)) = get_current_process().write().files.get_mut(&fd)
            {
                if arg & FD_CLOEXEC as usize != 0 {
                    mode.insert(FileMode::O_CLOEXEC);
                } else {
                    mode.remove(FileMode::O_CLOEXEC);
                }
            }

            return SystemError::EBADF.to_posix_errno() as isize;
        }
        _ => SystemError::ENOSYS.to_posix_errno() as isize,
    }
}

pub fn sys_getcwd(buf: &mut [u8]) -> isize {
    let current_process = get_current_process();
    let cwd = current_process.read().cwd.read().get_path();

    let cwd_bytes = cwd.as_bytes();
    let cwd_len = cwd_bytes.len();

    if cwd.len() + 1 > buf.len() {
        return SystemError::ENOMEM.to_posix_errno() as isize;
    }

    buf[..cwd_len].copy_from_slice(cwd_bytes);
    buf[cwd_len] = 0;

    buf.as_ptr() as isize
}
