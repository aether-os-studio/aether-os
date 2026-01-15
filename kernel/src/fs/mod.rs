use spin::Mutex;

use crate::{
    drivers::storage::BLOCK_DEVICES,
    fs::{ext2::Ext2FileSystem, vfs::fs::ArcFileSystem},
};

pub mod ext2;
pub mod vfs;

pub static ROOT_FILE_SYSTEM: Mutex<Option<ArcFileSystem>> = Mutex::new(None);

pub fn init() {
    for block_device in BLOCK_DEVICES.lock().iter() {
        if let Some(fs) = Ext2FileSystem::new(block_device.clone()) {
            *ROOT_FILE_SYSTEM.lock() = Some(fs);
            info!("Mount root file system OK");
            return;
        }
    }
    panic!("Failed to mount root");
}
