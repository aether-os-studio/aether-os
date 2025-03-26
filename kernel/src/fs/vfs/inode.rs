use alloc::{string::String, sync::Arc, vec::Vec};
use error::SystemError;
use spin::RwLock;

pub type InodeRef = Arc<RwLock<dyn Inode>>;

#[derive(PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum InodeTy {
    Dir = 0,
    File = 1,
    Device = 2,
    Symlink = 3,
}

#[repr(C)]
#[derive(Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct FileInfo {
    pub ty: InodeTy,
    pub name: String,
}

impl FileInfo {
    pub fn new(name: String, ty: InodeTy) -> Self {
        Self { name, ty }
    }
}

pub trait Inode: Sync + Send {
    fn when_mounted(&mut self, path: String, father: Option<InodeRef>);
    fn when_umounted(&mut self);

    fn get_path(&self) -> String;

    fn size(&self) -> usize {
        0
    }

    fn mount(&self, _node: InodeRef, _name: String) {
        unimplemented!()
    }

    fn read_at(&self, _offset: usize, _buf: &mut [u8]) -> Result<usize, SystemError> {
        Err(SystemError::ENOSYS)
    }
    fn write_at(&self, _offset: usize, _buf: &[u8]) -> Result<usize, SystemError> {
        Err(SystemError::ENOSYS)
    }

    fn open(&self, _name: String) -> Result<InodeRef, SystemError> {
        Err(SystemError::ENOSYS)
    }

    fn create(&self, _name: String, _ty: InodeTy) -> Result<InodeRef, SystemError> {
        Err(SystemError::ENOSYS)
    }

    fn list(&self) -> Vec<FileInfo> {
        Vec::new()
    }

    fn inode_type(&self) -> InodeTy {
        InodeTy::File
    }

    fn flush(&self) {}
}

pub fn mount_to(node: InodeRef, to: InodeRef, name: String) {
    to.read().mount(node.clone(), name.clone());
    node.write()
        .when_mounted(to.read().get_path() + &name + "/", Some(to.clone()));
}
