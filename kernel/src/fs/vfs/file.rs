use alloc::{sync::Arc, vec::Vec};
use spin::RwLock;

use crate::fs::vfs::{dirent::Dirent, fs::FileSystemTrait};

#[derive(Debug, Clone, Copy)]
pub enum FileType {
    Regular,
    Directory,
    Symlink,
    Fifo,
    CharDev,
    BlockDev,
    Socket,
}

pub trait FileTrait: Send + Sync + FileSystemTrait {
    fn size(&self) -> Option<usize>;

    fn read(&mut self, buf: &mut [u8], offset: usize) -> Option<usize>;
    fn write(&mut self, buf: &[u8], offset: usize) -> Option<usize>;

    fn readdir(&mut self) -> Option<Vec<Dirent>>;

    fn chmod(&mut self, mode: u16) -> Option<()>;
    fn chown(&mut self, uid: u32, gid: u32) -> Option<()>;
}

pub type ArcFile = Arc<RwLock<dyn FileTrait>>;
