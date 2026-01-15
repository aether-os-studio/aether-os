use alloc::string::String;

use crate::fs::vfs::file::FileType;

#[derive(Debug, Clone)]
pub struct Dirent {
    inode: u64,
    filetype: FileType,
    name: String,
}

impl Dirent {
    pub fn new(inode: u64, filetype: FileType, name: String) -> Self {
        Self {
            inode,
            filetype,
            name,
        }
    }

    pub fn inode(&self) -> u64 {
        self.inode
    }

    pub fn filetype(&self) -> FileType {
        self.filetype
    }

    pub fn name(&self) -> String {
        self.name.clone()
    }
}
