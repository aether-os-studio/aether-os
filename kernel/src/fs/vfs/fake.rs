use alloc::{collections::BTreeMap, string::String, sync::Arc, vec::Vec};
use error::SystemError;
use spin::RwLock;

use crate::ref_to_mut;

use super::inode::{FileInfo, Inode, InodeRef};

pub struct FakeFS {
    nodes: BTreeMap<String, InodeRef>,
    path: String,
}

impl FakeFS {
    pub fn new() -> InodeRef {
        let inode = Arc::new(RwLock::new(Self {
            nodes: BTreeMap::new(),
            path: String::new(),
        }));
        ref_to_mut(&*inode.read())
            .nodes
            .insert(".".into(), inode.clone());
        inode.clone()
    }
}

impl Inode for FakeFS {
    fn when_mounted(&mut self, path: String, father: Option<InodeRef>) {
        self.path.clear();
        self.path.push_str(path.as_str());
        if let Some(father) = father {
            self.nodes.insert("..".into(), father.clone());
        }
    }

    fn when_umounted(&mut self) {
        for (name, node) in self.nodes.iter() {
            if name != "." && name != ".." {
                node.write().when_umounted();
            }
        }
    }

    fn mount(&self, node: InodeRef, name: String) {
        ref_to_mut(self).nodes.insert(name, node);
    }

    fn get_path(&self) -> String {
        self.path.clone()
    }

    fn open(&self, name: String) -> Result<InodeRef, SystemError> {
        self.nodes.get(&name).cloned().ok_or(SystemError::ENOENT)
    }

    fn inode_type(&self) -> super::inode::InodeTy {
        super::inode::InodeTy::Dir
    }

    fn list(&self) -> alloc::vec::Vec<super::inode::FileInfo> {
        let mut vec = Vec::new();
        for (name, inode) in self.nodes.iter() {
            vec.push(FileInfo::new(name.clone(), inode.read().inode_type()));
        }
        vec
    }
}
