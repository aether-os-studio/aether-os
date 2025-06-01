use alloc::{string::String, sync::Arc};
use fat::Fat32Volume;
use spin::{Lazy, Mutex, RwLock};
use vfs::{IndexNode, IndexNodeRef, fake::FakeFS, partition::PartitionIndexNode};

use crate::{drivers::block::partition::PARTITION_DEVICES, print, serial_print, serial_println};

pub mod fat;
pub mod fd;
pub mod path_walk;
pub mod syscall;
pub mod vfs;

pub struct StdioIndexNode {
    path: String,
}

impl StdioIndexNode {
    pub fn new() -> IndexNodeRef {
        Arc::new(RwLock::new(StdioIndexNode {
            path: String::new(),
        }))
    }
}

impl IndexNode for StdioIndexNode {
    fn when_mounted(&mut self, path: String, father: Option<IndexNodeRef>) {
        self.path.clear();
        self.path.push_str(path.as_str());
    }

    fn when_umounted(&mut self) {}

    fn get_info(&self) -> vfs::IndexNodeInfo {
        let mut index_node_info = vfs::IndexNodeInfo::default();
        index_node_info.path = self.path.clone();
        index_node_info.inode_type = vfs::IndexNodeType::File;
        index_node_info
    }

    fn read_at(&self, _offset: usize, buf: &mut [u8]) -> crate::syscall::Result<usize> {
        Ok(buf.len())
    }

    fn write_at(&self, _offset: usize, buf: &[u8]) -> crate::syscall::Result<usize> {
        let str = unsafe { str::from_utf8_unchecked(buf) };
        serial_print!("{}", str);
        print!("{}", str);
        Ok(buf.len())
    }
}

pub static ROOT: Lazy<Mutex<IndexNodeRef>> = Lazy::new(|| Mutex::new(FakeFS::new()));

pub fn init() {
    for partition in PARTITION_DEVICES.lock().iter() {
        if let Some(root) = Fat32Volume::new(PartitionIndexNode::new(partition.clone())) {
            *ROOT.lock() = root;
            serial_println!("Mount root OK");
            break;
        }
    }
}
