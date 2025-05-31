use fat::Fat32Volume;
use spin::{Lazy, Mutex};
use vfs::{IndexNodeRef, fake::FakeFS, partition::PartitionIndexNode};

use crate::{drivers::block::partition::PARTITION_DEVICES, serial_println};

pub mod fat;
pub mod fd;
pub mod path_walk;
pub mod syscall;
pub mod vfs;

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
