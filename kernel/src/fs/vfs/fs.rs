use alloc::{string::String, sync::Arc};
use spin::RwLock;

use crate::fs::vfs::file::ArcFile;

pub trait FileSystemTrait: Send + Sync {
    fn lookup(&mut self, path: String, follow_symlink: bool) -> Option<ArcFile>;
}

pub type ArcFileSystem = Arc<RwLock<dyn FileSystemTrait>>;
