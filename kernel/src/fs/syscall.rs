use core::{ffi::CStr, hint::spin_loop};

use alloc::{
    string::{String, ToString},
    sync::Arc,
    vec::Vec,
};
use spin::RwLock;

use crate::{
    arch::nr::PollFd,
    errno::Errno,
    fs::vfs::pipe::PipeIndexNode,
    proc::{
        MAX_FD_NUM,
        signal::{SIG_SETMASK, sys_rt_sigprocmask},
    },
    syscall::{Result, check_user_overflows, slice_from_user, slice_from_user_mut},
    time::PosixTimeSpec,
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

    let poll_fds = unsafe { core::slice::from_raw_parts_mut(fds, nfds) };
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

pub const S_IFMT: usize = 00170000;
pub const S_IFSOCK: usize = 0140000;
pub const S_IFLNK: usize = 0120000;
pub const S_IFREG: usize = 0100000;
pub const S_IFBLK: usize = 0060000;
pub const S_IFDIR: usize = 0040000;
pub const S_IFCHR: usize = 0020000;
pub const S_IFIFO: usize = 0010000;
pub const S_ISUID: usize = 0004000;
pub const S_ISGID: usize = 0002000;
pub const S_ISVTX: usize = 0001000;

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
        (*stat_buf).st_mode = if inode.read().get_info().inode_type == IndexNodeType::Dir {
            S_IFDIR as u32
        } else {
            S_IFREG as u32
        };
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
        (*stat_buf).st_mode = if inode.read().get_info().inode_type == IndexNodeType::Dir {
            S_IFDIR as u32
        } else {
            S_IFREG as u32
        };
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

pub fn sys_chdir(ptr: usize) -> Result<usize> {
    let fd_manager = get_file_descriptor_manager().ok_or(Errno::ENOENT)?;

    let str = unsafe { CStr::from_ptr(ptr as *const core::ffi::c_char) }
        .to_str()
        .or(Err(Errno::EINVAL))?;

    if check_user_overflows(ptr, str.len()) {
        return Err(Errno::EFAULT);
    }

    fd_manager.change_cwd(str.to_string());

    Ok(0)
}

pub fn sys_getcwd(ptr: usize, size: usize) -> Result<usize> {
    let fd_manager = get_file_descriptor_manager().ok_or(Errno::ENOENT)?;
    let mut cwd = fd_manager.get_cwd();
    cwd.insert(0, '/');

    let len = cwd.len().min(size);

    let buf = slice_from_user_mut(ptr as *mut u8, len).ok_or(Errno::EFAULT)?;
    buf.copy_from_slice(&cwd.as_bytes()[..len]);

    Ok(len)
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

pub struct WeirdPselect6 {
    ss: *const usize,
    ss_len: usize,
}

pub struct FdSet {
    fd_bits: [u64; 1024 / 8 / size_of::<u64>()],
}

pub fn sys_select(
    nfds: usize,
    readfds_ptr: usize,
    writefds_ptr: usize,
    exceptfds_ptr: usize,
    timeout: *const PosixTimeSpec,
) -> Result<usize> {
    let mut poll_fds = Vec::with_capacity(nfds);

    if readfds_ptr != 0 {
        for fd in bitmap_iter(
            slice_from_user(readfds_ptr as *const u64, 1).ok_or(Errno::EFAULT)?,
            nfds,
        ) {
            poll_fds.push(PollFd {
                fd: fd as i32,
                events: POLLIN as i16,
                revents: 0,
            });
        }
    }

    if writefds_ptr != 0 {
        for fd in bitmap_iter(
            slice_from_user(writefds_ptr as *const u64, 1).ok_or(Errno::EFAULT)?,
            nfds,
        ) {
            poll_fds.push(PollFd {
                fd: fd as i32,
                events: POLLOUT as i16,
                revents: 0,
            });
        }
    }

    if exceptfds_ptr != 0 {
        for fd in bitmap_iter(
            slice_from_user(exceptfds_ptr as *const u64, 1).ok_or(Errno::EFAULT)?,
            nfds,
        ) {
            poll_fds.push(PollFd {
                fd: fd as i32,
                events: POLLPRI as i16,
                revents: 0,
            });
        }
    }

    let timeout_ms = if !timeout.is_null() {
        let ts = slice_from_user(timeout, 1).ok_or(Errno::EFAULT)?;
        (ts[0].tv_sec as isize * 1000 + ts[0].tv_nsec as isize / 1_000_000) as isize
    } else {
        -1
    };

    if readfds_ptr != 0 {
        clear_bitmap(
            slice_from_user_mut(readfds_ptr as *mut u64, FD_SET_SIZE).ok_or(Errno::EFAULT)?,
        );
    }
    if writefds_ptr != 0 {
        clear_bitmap(
            slice_from_user_mut(writefds_ptr as *mut u64, FD_SET_SIZE).ok_or(Errno::EFAULT)?,
        );
    }
    if exceptfds_ptr != 0 {
        clear_bitmap(
            slice_from_user_mut(exceptfds_ptr as *mut u64, FD_SET_SIZE).ok_or(Errno::EFAULT)?,
        );
    }

    let ready = sys_poll(poll_fds.as_mut_ptr(), poll_fds.len(), timeout_ms)?;

    let mut count = 0;
    for pf in &poll_fds {
        if pf.revents == 0 {
            continue;
        }

        if pf.events as usize & POLLIN != 0
            && pf.revents as usize & ((POLLIN | POLLERR | POLLHUP) as usize) != 0
        {
            if readfds_ptr != 0 {
                set_bitmap_bit(
                    slice_from_user_mut(readfds_ptr as *mut u64, FD_SET_SIZE)
                        .ok_or(Errno::EFAULT)?,
                    pf.fd as usize,
                );
            }
            count += 1;
        }
        if pf.events as usize & POLLOUT != 0
            && pf.revents as usize & ((POLLOUT | POLLERR | POLLHUP) as usize) != 0
        {
            if writefds_ptr != 0 {
                set_bitmap_bit(
                    slice_from_user_mut(writefds_ptr as *mut u64, FD_SET_SIZE)
                        .ok_or(Errno::EFAULT)?,
                    pf.fd as usize,
                );
            }
            count += 1;
        }
        if pf.events as usize & POLLPRI != 0
            && pf.revents as usize & ((POLLPRI | POLLERR) as usize) != 0
        {
            if exceptfds_ptr != 0 {
                set_bitmap_bit(
                    slice_from_user_mut(exceptfds_ptr as *mut u64, FD_SET_SIZE)
                        .ok_or(Errno::EFAULT)?,
                    pf.fd as usize,
                );
            }
            count += 1;
        }
    }

    Ok(count)
}

const FD_SET_SIZE: usize = 1024 / 64;
fn bitmap_iter(set: &[u64], nfds: usize) -> impl Iterator<Item = usize> + '_ {
    (0..nfds).filter(move |&i| (set[i / 64] & (1 << (i % 64))) != 0)
}

fn clear_bitmap(set: &mut [u64]) {
    for item in set.iter_mut() {
        *item = 0;
    }
}

fn set_bitmap_bit(set: &mut [u64], fd: usize) {
    let idx = fd / 64;
    let bit = fd % 64;
    if idx < set.len() {
        set[idx] |= 1 << bit;
    }
}

pub fn sys_pselect6(
    nfds: usize,
    readfds: usize,
    writefds: usize,
    exceptfds: usize,
    timeout: *const PosixTimeSpec,
    wired_pselect6: *const WeirdPselect6,
) -> Result<usize> {
    if check_user_overflows(timeout as usize, size_of::<PosixTimeSpec>()) {
        return Err(Errno::EFAULT);
    }
    if wired_pselect6.is_null()
        || check_user_overflows(wired_pselect6 as usize, size_of::<WeirdPselect6>())
    {
        return Err(Errno::EFAULT);
    }

    let sigset_size = unsafe { (*wired_pselect6).ss_len };
    let sigmask = unsafe { (*wired_pselect6).ss };

    if sigset_size < size_of::<usize>() {
        return Err(Errno::EINVAL);
    }

    let mut sig: usize = 0;
    if !sigmask.is_null() {
        sys_rt_sigprocmask(
            SIG_SETMASK,
            sigmask as *const u64,
            &mut sig as *mut usize as *mut u64,
            sigset_size,
        )?;
    }

    let ret = sys_select(nfds, readfds, writefds, exceptfds, timeout)?;

    if !sigmask.is_null() {
        sys_rt_sigprocmask(
            SIG_SETMASK,
            &sig as *const usize as *const u64,
            core::ptr::null_mut(),
            sigset_size,
        )?;
    }

    Ok(ret)
}

pub fn sys_pipe(pipefd: &mut [i32]) -> Result<usize> {
    let (read_end, write_end) = PipeIndexNode::new_pair();

    if let Some(fd_manager) = get_file_descriptor_manager() {
        let read_fd = fd_manager.add_inode(Arc::new(RwLock::new(read_end)), OpenMode::Read);
        let write_fd = fd_manager.add_inode(Arc::new(RwLock::new(write_end)), OpenMode::Write);

        pipefd[0] = read_fd as i32;
        pipefd[1] = write_fd as i32;

        Ok(0)
    } else {
        Err(Errno::ENOENT)
    }
}

pub fn sys_readlink(path: *const core::ffi::c_char, buf: &mut [u8]) -> Result<usize> {
    let path = unsafe { CStr::from_ptr(path) }
        .to_str()
        .or(Err(Errno::EINVAL))?
        .to_string();

    let len = buf.len().min(path.len());

    buf[..len].copy_from_slice(&path.as_bytes()[..len]);

    Ok(len)
}
