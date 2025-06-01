use core::any::Any;

use alloc::{string::String, sync::Arc};
use spin::RwLock;

use crate::{errno::Errno, syscall::Result};

pub type IndexNodeRef = Arc<RwLock<dyn IndexNode>>;

pub mod fake;
pub mod partition;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum IndexNodeType {
    #[default]
    File = 1,
    Dir,
    Symlink,
    Dev,
    Socket,
}

#[derive(Debug, Clone, Default)]
pub struct IndexNodeInfo {
    pub path: String,
    pub size: usize,
    pub inode_id: usize,
    pub inode_type: IndexNodeType,
}

pub struct Dirent {
    d_ino: usize,
    d_off: usize,
    d_reclen: u16,
    d_type: u8,
    d_name: [u8; 256],
}

pub trait IndexNode: Sync + Send + Any {
    /// 不论什么，必须实现下面三个
    fn when_mounted(&mut self, path: String, father: Option<IndexNodeRef>);
    fn when_umounted(&mut self);
    fn get_info(&self) -> IndexNodeInfo;

    fn mount(&self, _node: IndexNodeRef, _name: String) -> Result<()> {
        Err(Errno::ENOSYS)
    }

    fn read_at(&self, _offset: usize, _buf: &mut [u8]) -> Result<usize> {
        Err(Errno::ENOSYS)
    }
    fn write_at(&self, _offset: usize, _buf: &[u8]) -> Result<usize> {
        Err(Errno::ENOSYS)
    }

    fn open(&self, _name: String) -> Result<IndexNodeRef> {
        Err(Errno::ENOSYS)
    }

    fn create(&self, _name: String, _ty: IndexNodeType) -> Result<IndexNodeRef> {
        Err(Errno::ENOSYS)
    }

    fn rename(&self, new_name: String) -> Result<()> {
        Err(Errno::ENOSYS)
    }

    fn symlink(&self, target: String) -> Result<()> {
        Err(Errno::ENOSYS)
    }
    fn link(&self) -> Result<()> {
        Err(Errno::ENOSYS)
    }
    fn unlink(&self) -> Result<()> {
        Err(Errno::ENOSYS)
    }

    fn poll(&self, event: usize) -> Result<usize> {
        Ok(event)
    }
    fn ioctl(&self, _cmd: usize, _arg: usize) -> Result<usize> {
        Err(Errno::ENOSYS)
    }

    fn dup(&self) -> Result<IndexNodeRef> {
        Err(Errno::ENOSYS)
    }

    fn getdents(&self, dents: &mut [Dirent]) -> Result<()> {
        Err(Errno::ENOSYS)
    }
}
