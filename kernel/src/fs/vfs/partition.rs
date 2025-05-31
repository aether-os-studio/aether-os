use alloc::{string::String, sync::Arc};
use spin::RwLock;

use crate::drivers::block::partition::PartitionDevice;

use super::{IndexNode, IndexNodeInfo, IndexNodeRef, IndexNodeType};

pub struct PartitionIndexNode {
    partition: PartitionDevice,
    path: String,
}

impl PartitionIndexNode {
    pub fn new(partition: PartitionDevice) -> IndexNodeRef {
        Arc::new(RwLock::new(PartitionIndexNode {
            partition,
            path: String::new(),
        }))
    }
}

impl IndexNode for PartitionIndexNode {
    fn when_mounted(&mut self, path: String, father: Option<IndexNodeRef>) {
        self.path.clear();
        self.path.push_str(path.as_str());
    }

    fn when_umounted(&mut self) {}

    fn get_info(&self) -> super::IndexNodeInfo {
        let mut info = IndexNodeInfo::default();
        info.path = info.path.clone();
        info.inode_type = IndexNodeType::File;
        info.clone()
    }

    fn read_at(&self, offset: usize, buf: &mut [u8]) -> crate::syscall::Result<usize> {
        self.partition.read_at(offset, buf)
    }

    fn write_at(&self, offset: usize, buf: &[u8]) -> crate::syscall::Result<usize> {
        self.partition.write_at(offset, buf)
    }
}
