use alloc::string::ToString;
use crossbeam_queue::ArrayQueue;
use error::SystemError;
use goto::gpoint;
use spin::Lazy;

use crate::{context::get_current_process, fs::vfs::get_inode_by_path};

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
        .insert(next_fd, (inode.clone(), true, 0));

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
        1 | 2 => {
            let str = unsafe { str::from_utf8_unchecked(data) };
            crate::serial_print!("{}", str);
            crate::print!("{}", str);
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
