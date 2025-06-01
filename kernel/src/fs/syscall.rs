use core::{ffi::CStr, hint::spin_loop};

use alloc::string::ToString;

use crate::{
    arch::nr::PollFd,
    errno::Errno,
    syscall::{Result, slice_from_user_mut},
};

use super::{
    fd::{OpenMode, get_file_descriptor_manager},
    path_walk::{get_inode_by_path, ref_to_mut},
};

pub fn sys_open(
    path: *const core::ffi::c_char,
    flags: core::ffi::c_int,
    mode: core::ffi::c_int,
) -> Result<usize> {
    let path = unsafe { CStr::from_ptr(path).to_str().unwrap() }.to_string();

    let current_file_descriptor_manager = get_file_descriptor_manager().ok_or(Errno::ENOENT)?;

    let inode = if path.starts_with("/") {
        get_inode_by_path(path.clone()).ok_or(Errno::ENOENT)?
    } else {
        get_inode_by_path(alloc::format!(
            "{}{}",
            current_file_descriptor_manager.get_cwd(),
            path.clone()
        ))
        .ok_or(Errno::ENOENT)?
    };

    let file_descriptor =
        current_file_descriptor_manager.add_inode(inode, super::fd::OpenMode::ReadWrite); // TODO: mode

    Ok(file_descriptor)
}

pub fn sys_close(fd: core::ffi::c_int) -> Result<usize> {
    let current_file_descriptor_manager = get_file_descriptor_manager().ok_or(Errno::ENOENT)?;

    ref_to_mut(current_file_descriptor_manager.as_ref())
        .file_descriptors
        .remove(&(fd as usize))
        .ok_or(Errno::EBADF)?;

    Ok(0)
}

pub fn sys_read(fd: usize, buf: &mut [u8]) -> Result<usize> {
    let current_file_descriptor_manager = get_file_descriptor_manager();
    if let None = current_file_descriptor_manager {
        return Err(Errno::ENOENT);
    }
    let current_file_descriptor_manager = current_file_descriptor_manager.unwrap();

    if let Some((inode, _, offset)) = current_file_descriptor_manager.file_descriptors.get(&fd) {
        inode.read().read_at(*offset, buf).or(Err(Errno::EBADF))
    } else {
        Err(Errno::ENOENT)
    }
}

pub fn sys_write(fd: usize, buf: &[u8]) -> Result<usize> {
    if let Some(current_file_descriptor_manager) = get_file_descriptor_manager() {
        if let Some((inode, mode, offset)) =
            current_file_descriptor_manager.file_descriptors.get(&fd)
        {
            match mode {
                OpenMode::Write | OpenMode::ReadWrite => {
                    inode.read().write_at(*offset, buf).or(Err(Errno::EBADF))
                }

                _ => Err(Errno::EPERM),
            }
        } else {
            Err(Errno::ENOENT)
        }
    } else {
        Err(Errno::ENOENT)
    }
}

pub const POLLIN: usize = 0x001;
pub const POLLPRI: usize = 0x002;
pub const POLLOUT: usize = 0x004;
pub const POLLERR: usize = 0x008;
pub const POLLHUP: usize = 0x010;
pub const POLLNVAL: usize = 0x020;

pub fn sys_poll(fds: *mut PollFd, nfds: usize, timeout: isize) -> Result<usize> {
    let current_fd_manager = super::fd::get_file_descriptor_manager().ok_or(Errno::ENOENT)?;
    let mut ready = 0;

    if let Some(poll_fds) = slice_from_user_mut(fds, nfds) {
        loop {
            for poll_fd in poll_fds.iter_mut() {
                if let Some((inode, _, _)) = current_fd_manager
                    .file_descriptors
                    .get(&(poll_fd.fd as usize))
                {
                    match inode.read().poll(poll_fd.events as usize) {
                        Ok(revents) => {
                            poll_fd.revents = revents as i16;
                            ready += 1;
                        }
                        Err(_) => poll_fd.revents = POLLERR as i16,
                    }
                } else {
                    poll_fd.revents = POLLNVAL as i16;
                }
            }
            if ready > 0 || timeout == 0 {
                return Ok(ready);
            }

            spin_loop();
        }
    } else {
        Err(Errno::EFAULT)
    }
}
