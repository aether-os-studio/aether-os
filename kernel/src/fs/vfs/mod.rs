use alloc::string::{String, ToString};
use dev::ROOT_PARTITION;
use error::SystemError;
use fake::FakeFS;
use inode::{InodeRef, mount_to};
use proc::ProcFS;
use spin::{Lazy, Mutex};
use uuid::Uuid;

use crate::{ref_to_mut, unwind::EXE_REQUEST};

use super::fat::Fat32Volume;

pub mod cache;
pub mod dev;
pub mod fake;
pub mod fcntl;
pub mod inode;
pub mod proc;

pub static ROOT: Lazy<Mutex<InodeRef>> = Lazy::new(|| Mutex::new(FakeFS::new()));

pub fn get_inode_by_path(path: String) -> Result<InodeRef, SystemError> {
    let root = ROOT.lock().clone();

    let path = path.split("/");

    let node = root;

    for path_node in path {
        if path_node.len() > 0 {
            let child = node.read().open(String::from(path_node))?;
            core::mem::drop(core::mem::replace(ref_to_mut(&node), child));
        }
    }

    Ok(node.clone())
}

pub fn get_root_partition_uuid() -> Uuid {
    let kernel_file_response = EXE_REQUEST.get_response().unwrap();
    Uuid::from(kernel_file_response.file().gpt_partition_id().unwrap())
}

pub fn init() {
    ROOT.lock().write().when_mounted("/".to_string(), None);

    dev::init();

    let root_partition = ROOT_PARTITION.lock().clone().unwrap().clone();
    let root_fs = Fat32Volume::new(root_partition.clone());

    let dev_fs = ROOT.lock().read().open("dev".into()).unwrap();

    *ROOT.lock() = root_fs.clone();

    root_fs.write().when_mounted("/".to_string(), None);
    dev_fs.write().when_umounted();
    mount_to(dev_fs.clone(), root_fs.clone(), "dev".to_string());

    let procfs = ProcFS::new();
    mount_to(procfs.clone(), root_fs.clone(), "proc".to_string());
}
