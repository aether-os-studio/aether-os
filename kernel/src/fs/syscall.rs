use core::{ffi::CStr, hint::spin_loop};

use alloc::string::{String, ToString};

use crate::{
    arch::nr::PollFd,
    errno::Errno,
    proc::MAX_FD_NUM,
    syscall::{Result, check_user_overflows, slice_from_user_mut},
};

use super::{
    AtFlags, F_DUPFD, F_GETFD, F_GETFL, F_SETFD, F_SETFL, PosixStat,
    fd::{OpenMode, get_file_descriptor_manager},
    path_walk::{get_inode_by_path, ref_to_mut},
    vfs::{
        IndexNodeType,
        iov::{IoVec, IoVecs},
    },
};

pub fn resolve_pathname(dirfd: usize, path: &str) -> Result<String> {
    let ret_path;
    if path.as_bytes()[0] != b'/' {
        if dirfd != AtFlags::AT_FDCWD.bits() as usize {
            let fd_manager = get_file_descriptor_manager().ok_or(Errno::ENOENT)?;

            let (file, _, _) = fd_manager
                .file_descriptors
                .get(&dirfd)
                .ok_or(Errno::EBADF)?;

            if file.read().get_info().inode_type != IndexNodeType::Dir {
                return Err(Errno::ENOTDIR);
            }

            ret_path = String::from(path);
        } else {
            let mut cwd = get_file_descriptor_manager()
                .ok_or(Errno::ENOENT)?
                .get_cwd();
            cwd.push('/');
            cwd.push_str(path);
            ret_path = cwd;
        }
    } else {
        ret_path = String::from(path);
    }

    return Ok(ret_path);
}

pub fn sys_open(
    path: *const core::ffi::c_char,
    flags: core::ffi::c_int,
    mode: core::ffi::c_int,
) -> Result<usize> {
    let path = unsafe { CStr::from_ptr(path) }
        .to_str()
        .or(Err(Errno::EINVAL))?
        .to_string();

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

pub fn sys_dup(fd: usize) -> Result<usize> {
    let fd_manager = get_file_descriptor_manager().ok_or(Errno::ENOENT)?;
    if let Some((old, mode, _)) = fd_manager.file_descriptors.get(&fd) {
        let new = fd_manager.add_inode(old.clone(), mode.clone());
        return Ok(new);
    }
    Err(Errno::EBADF)
}

pub fn sys_fcntl(fd: usize, cmd: usize, arg: usize) -> Result<usize> {
    match cmd {
        F_DUPFD => sys_dup(fd),
        F_SETFD => Ok(0),
        F_GETFD => Ok(0),
        F_GETFL => Ok(0),
        F_SETFL => Ok(0),
        _ => Err(Errno::ENOSYS),
    }
}

// todo: inode_id

pub fn sys_fstat(fd: usize, buf: usize) -> Result<usize> {
    let fd_manager = get_file_descriptor_manager().ok_or(Errno::ENOENT)?;
    let (inode, mode, offset) = fd_manager.file_descriptors.get(&fd).ok_or(Errno::EBADF)?;

    let stat_buf = buf as *mut PosixStat;

    let info = inode.read().get_info();

    static mut INO: usize = 1;

    unsafe {
        (*stat_buf) = PosixStat::default();
        (*stat_buf).st_ino = INO;
        INO += 1;
        (*stat_buf).st_size = info.size as isize;
    }

    Ok(0)
}

pub fn sys_stat(path: *const core::ffi::c_char, buf: usize) -> Result<usize> {
    let path = unsafe { CStr::from_ptr(path) }
        .to_str()
        .or(Err(Errno::EINVAL))?;

    let inode = get_inode_by_path(path.to_string()).ok_or(Errno::ENOENT)?;

    let stat_buf = buf as *mut PosixStat;

    let info = inode.read().get_info();

    static mut INO: usize = 1;

    unsafe {
        (*stat_buf) = PosixStat::default();
        (*stat_buf).st_ino = INO;
        INO += 1;
        (*stat_buf).st_size = info.size as isize;
    }

    Ok(0)
}

pub fn sys_readv(fd: usize, iovec: *const IoVec, count: usize) -> Result<usize> {
    let iovecs = unsafe { IoVecs::from_user(iovec, count, true) }?;

    let mut data = alloc::vec![0; iovecs.total_len()];

    let len = sys_read(fd, &mut data)?;

    iovecs.scatter(&data[..len]);

    return Ok(len);
}

pub fn sys_writev(fd: usize, iovec: *const IoVec, count: usize) -> Result<usize> {
    let iovecs = unsafe { IoVecs::from_user(iovec, count, false) }?;
    let data = iovecs.gather();
    sys_write(fd, &data)
}

pub fn sys_getcwd(ptr: usize, size: usize) -> Result<usize> {
    let fd_manager = get_file_descriptor_manager().ok_or(Errno::ENOENT)?;
    let cwd = fd_manager.get_cwd();

    let len = cwd.len().min(size);

    let buf = slice_from_user_mut(ptr as *mut u8, len).ok_or(Errno::EFAULT)?;
    buf.copy_from_slice(&cwd.as_bytes()[..len]);

    Ok(0)
}

pub fn sys_faccessat(dirfd: usize, pathname: usize, mode: usize) -> Result<usize> {
    let cstr = unsafe { CStr::from_ptr(pathname as *const core::ffi::c_char) }
        .to_str()
        .or(Err(Errno::EINVAL))?;

    let path = resolve_pathname(dirfd, cstr)?;

    let inode = get_inode_by_path(path.to_string()).ok_or(Errno::ENOENT)?;
    drop(inode);

    Ok(0)
}

pub fn sys_faccessat2(dirfd: usize, pathname: usize, mode: usize, flags: usize) -> Result<usize> {
    let cstr = unsafe { CStr::from_ptr(pathname as *const core::ffi::c_char) }
        .to_str()
        .or(Err(Errno::EINVAL))?;

    let path = resolve_pathname(dirfd, cstr)?;

    let inode = get_inode_by_path(path.to_string()).ok_or(Errno::ENOENT)?;
    drop(inode);

    Ok(0)
}

pub const SEEK_SET: usize = 0;
pub const SEEK_CUR: usize = 1;
pub const SEEK_END: usize = 2;
pub const SEEK_MAX: usize = 3;

pub fn sys_lseek(fd: usize, off: usize, whence: usize) -> Result<usize> {
    let fd_manager = get_file_descriptor_manager().ok_or(Errno::ENOENT)?;
    if let Some((ino, _, offset)) = ref_to_mut(fd_manager.as_ref())
        .file_descriptors
        .get_mut(&fd)
    {
        match whence {
            SEEK_SET => *offset = off,
            SEEK_CUR => *offset = *offset + off,
            SEEK_END => *offset = (ino.read().get_info().size) - off,
            _ => return Err(Errno::EINVAL),
        }
    }
    Err(Errno::EBADF)
}

pub struct RLimit {
    curr: usize,
    max: usize,
}

pub fn sys_getrlimit(resource: usize, ptr: usize) -> Result<usize> {
    if check_user_overflows(ptr, size_of::<RLimit>()) {
        return Err(Errno::EFAULT);
    }
    match resource {
        7 => {
            let rlim = ptr as *mut RLimit;
            unsafe {
                (*rlim).curr = MAX_FD_NUM;
                (*rlim).max = MAX_FD_NUM;
            }
            Ok(0)
        }
        _ => return Err(Errno::EINVAL),
    }
}

pub fn sys_prlimit64(resource: usize, new: usize, old: usize) -> Result<usize> {
    if old != 0 {
        sys_getrlimit(resource, old)?;
    }

    if new != 0 {
        // todo
    }

    Ok(0)
}

pub fn sys_ioctl(fd: usize, cmd: usize, arg: usize) -> Result<usize> {
    let fd_manager = get_file_descriptor_manager().ok_or(Errno::ENOENT)?;
    if let Some((ino, _, offset)) = fd_manager.file_descriptors.get(&fd) {
        return ino.read().ioctl(cmd, arg);
    }
    Err(Errno::EBADF)
}
