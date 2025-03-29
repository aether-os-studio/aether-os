use alloc::{
    collections::btree_map::BTreeMap,
    string::{String, ToString},
    sync::Arc,
};
use error::SystemError;
use spin::RwLock;

use crate::{
    context::get_current_process,
    fs::vfs::inode::{Inode, InodeRef, InodeTy},
};

use super::map::ProcMapFS;

pub struct ProcSelfFS {
    path: String,
    nodes: BTreeMap<String, InodeRef>,
}

impl ProcSelfFS {
    pub fn new() -> InodeRef {
        Arc::new(RwLock::new(ProcSelfFS {
            path: String::new(),
            nodes: BTreeMap::new(),
        }))
    }
}

impl Inode for ProcSelfFS {
    fn when_mounted(&mut self, path: String, father: Option<InodeRef>) {
        self.path.clear();
        self.path.push_str(path.as_str());
        if let Some(father) = father {
            self.nodes.insert("..".to_string(), father);
        }
    }

    fn when_umounted(&mut self) {}

    fn get_path(&self) -> String {
        self.path.clone()
    }

    fn open(&self, name: String) -> Result<InodeRef, SystemError> {
        if name == "maps" {
            let proc_map_fs = ProcMapFS::new(get_current_process().clone());
            return Ok(proc_map_fs);
        }
        return Err(SystemError::ENOENT);
    }

    fn inode_type(&self) -> InodeTy {
        InodeTy::Dir
    }
}
