use alloc::string::ToString;
use error::SystemError;

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

pub fn sys_read(fd: usize, data: &mut [u8]) -> isize {
    match fd {
        0 => 0,
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
            crate::serial_println!("{}", str);
            crate::println!("{}", str);
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
