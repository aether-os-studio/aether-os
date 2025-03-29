use alloc::{
    collections::btree_map::BTreeMap,
    string::{String, ToString},
    sync::Arc,
};
use error::SystemError;
use selffs::ProcSelfFS;
use spin::RwLock;

use super::inode::{Inode, InodeRef, InodeTy};

pub mod map;
pub mod selffs;

pub struct ProcFS {
    nodes: BTreeMap<String, InodeRef>,
    path: String,
}

impl ProcFS {
    pub fn new() -> InodeRef {
        let this = Arc::new(RwLock::new(Self {
            nodes: BTreeMap::new(),
            path: String::new(),
        }));

        this.write()
            .nodes
            .insert("self".to_string(), ProcSelfFS::new());

        this
    }
}

impl Inode for ProcFS {
    fn when_mounted(&mut self, path: String, father: Option<InodeRef>) {
        self.path.clear();
        self.path.push_str(path.as_str());
        if let Some(father) = father {
            self.nodes.insert("..".to_string(), father);
        }
    }

    fn when_umounted(&mut self) {
        self.nodes.clear();
    }

    fn get_path(&self) -> String {
        self.path.clone()
    }

    fn open(&self, name: String) -> Result<InodeRef, SystemError> {
        if let Some(inode) = self.nodes.get(&name) {
            return Ok(inode.clone());
        }

        Err(SystemError::ENOENT)
    }

    fn inode_type(&self) -> InodeTy {
        InodeTy::Dir
    }
}
