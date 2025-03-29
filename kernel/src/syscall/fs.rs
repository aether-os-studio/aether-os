use alloc::{
    string::{String, ToString},
    sync::Arc,
};
use crossbeam_queue::ArrayQueue;
use error::SystemError;
use goto::gpoint;
use spin::{Lazy, RwLock};

use crate::{
    context::get_current_process,
    fs::vfs::{
        fcntl::{FD_CLOEXEC, FcntlCommand},
        get_inode_by_path,
        inode::{FileMode, Inode, InodeRef},
    },
    time::PosixTimeSpec,
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

    return next_fd as isize;
}

pub fn sys_lseek(fd: usize, d_offset: usize, _whence: usize) -> isize {
    if let Some((_, _, offset)) = get_current_process().write().files.get_mut(&fd) {
        *offset = d_offset;
        return *offset as isize;
    } else {
        return SystemError::EBADF.to_posix_errno() as isize;
    }
}

const KEYCODE_QUEUE_SIZE: usize = 128;

pub static KEYCODE_QUEUE: Lazy<ArrayQueue<u8>> = Lazy::new(|| ArrayQueue::new(KEYCODE_QUEUE_SIZE));

pub struct StdioInode {
    path: String,
}

impl StdioInode {
    pub fn new() -> InodeRef {
        Arc::new(RwLock::new(StdioInode {
            path: String::new(),
        }))
    }
}

impl Inode for StdioInode {
    fn when_mounted(&mut self, path: String, _father: Option<InodeRef>) {
        self.path.clear();
        self.path.push_str(path.as_str());
    }

    fn when_umounted(&mut self) {}

    fn get_path(&self) -> String {
        self.path.clone()
    }

    fn read_at(&self, _offset: usize, data: &mut [u8]) -> Result<usize, SystemError> {
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
        Ok(write)
    }

    fn write_at(&self, _offset: usize, buf: &[u8]) -> Result<usize, SystemError> {
        let s = unsafe { str::from_utf8_unchecked(buf) };

        crate::serial_print!("{}", s);
        crate::print!("{}", s);

        Ok(s.len())
    }
}

pub fn sys_read(fd: usize, data: &mut [u8]) -> isize {
    if let Some((node, _, offset)) = get_current_process().read().files.get(&fd) {
        let result = node.read().read_at(*offset, data);
        if let Err(err) = result {
            return err.to_posix_errno() as isize;
        }
        result.unwrap() as isize
    } else {
        return SystemError::EBADF.to_posix_errno() as isize;
    }
}

pub fn sys_write(fd: usize, data: &[u8]) -> isize {
    if let Some((node, _, offset)) = get_current_process().read().files.get(&fd) {
        let result = node.read().write_at(*offset, data);
        if let Err(err) = result {
            return err.to_posix_errno() as isize;
        }
        result.unwrap() as isize
    } else {
        return SystemError::EBADF.to_posix_errno() as isize;
    }
}

pub fn sys_close(fd: usize) -> isize {
    if let Some(_) = get_current_process().write().files.remove(&fd) {
        return 0;
    }
    SystemError::EBADF.to_posix_errno() as isize
}

pub fn sys_fcntl(fd: usize, cmd: FcntlCommand, arg: usize) -> isize {
    match cmd {
        FcntlCommand::DupFd => {
            if let Some((inode, mode, offset)) = get_current_process().write().files.get_mut(&fd) {
                let next_fd = get_current_process()
                    .read()
                    .next_fd
                    .fetch_add(1, core::sync::atomic::Ordering::SeqCst);

                get_current_process()
                    .write()
                    .files
                    .insert(next_fd, (inode.clone(), mode.clone(), *offset));

                return next_fd as isize;
            }

            return SystemError::EBADF.to_posix_errno() as isize;
        }
        FcntlCommand::DupFdCloexec => {
            if let Some((inode, mode, offset)) = get_current_process().write().files.get_mut(&fd) {
                let next_fd = get_current_process()
                    .read()
                    .next_fd
                    .fetch_add(1, core::sync::atomic::Ordering::SeqCst);

                mode.insert(FileMode::O_CLOEXEC);

                get_current_process()
                    .write()
                    .files
                    .insert(next_fd, (inode.clone(), mode.clone(), *offset));

                return next_fd as isize;
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
        FcntlCommand::SetFd => {
            if let Some((_inode, mode, _offset)) = get_current_process().write().files.get_mut(&fd)
            {
                mode.insert(FileMode::O_CLOEXEC);
                return 0;
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

#[repr(C)]
#[derive(Clone, Copy)]
/// # 文件信息结构体
pub struct PosixKstat {
    /// 硬件设备ID
    dev_id: u64,
    /// inode号
    inode: u64,
    /// 硬链接数
    nlink: u64,
    /// 文件权限
    mode: FileMode,
    /// 所有者用户ID
    uid: i32,
    /// 所有者组ID
    gid: i32,
    /// 设备ID
    rdev: i64,
    /// 文件大小
    size: i64,
    /// 文件系统块大小
    blcok_size: i64,
    /// 分配的512B块数
    blocks: u64,
    /// 最后访问时间
    atime: PosixTimeSpec,
    /// 最后修改时间
    mtime: PosixTimeSpec,
    /// 最后状态变化时间
    ctime: PosixTimeSpec,
    /// 用于填充结构体大小的空白数据
    pub _pad: [i8; 24],
}
impl PosixKstat {
    fn new() -> Self {
        Self {
            inode: 0,
            dev_id: 0,
            mode: FileMode::empty(),
            nlink: 0,
            uid: 0,
            gid: 0,
            rdev: 0,
            size: 0,
            atime: PosixTimeSpec {
                tv_sec: 0,
                tv_nsec: 0,
            },
            mtime: PosixTimeSpec {
                tv_sec: 0,
                tv_nsec: 0,
            },
            ctime: PosixTimeSpec {
                tv_sec: 0,
                tv_nsec: 0,
            },
            blcok_size: 0,
            blocks: 0,
            _pad: Default::default(),
        }
    }
}

pub fn do_fstat(fd: usize) -> Result<PosixKstat, SystemError> {
    if let Some((inode, _, _)) = get_current_process().read().files.get(&fd) {
        let mut kstat = PosixKstat::new();

        kstat.size = inode.read().size() as i64;

        return Ok(kstat);
    }

    Err(SystemError::EBADF)
}
