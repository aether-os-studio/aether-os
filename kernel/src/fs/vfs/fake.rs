use alloc::{collections::BTreeMap, string::String, sync::Arc};
use spin::RwLock;

use crate::{errno::Errno, fs::path_walk::ref_to_mut, syscall::Result};

use super::{IndexNode, IndexNodeInfo, IndexNodeRef, IndexNodeType};

pub struct FakeFS {
    nodes: BTreeMap<String, IndexNodeRef>,
    path: String,
}

impl FakeFS {
    pub fn new() -> IndexNodeRef {
        let inode = Arc::new(RwLock::new(Self {
            nodes: BTreeMap::new(),
            path: String::new(),
        }));
        inode.write().nodes.insert(".".into(), inode.clone());
        inode
    }
}

impl IndexNode for FakeFS {
    fn when_mounted(&mut self, path: String, father: Option<IndexNodeRef>) {
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

    fn mount(&self, node: IndexNodeRef, name: String) -> Result<()> {
        ref_to_mut(self).nodes.insert(name, node);
        Ok(())
    }

    fn get_info(&self) -> IndexNodeInfo {
        let mut info = IndexNodeInfo::default();
        info.path = info.path.clone();
        info.inode_type = IndexNodeType::Dir;
        info.clone()
    }

    fn open(&self, name: String) -> Result<IndexNodeRef> {
        self.nodes.get(&name).cloned().ok_or(Errno::ENOENT)
    }
}
