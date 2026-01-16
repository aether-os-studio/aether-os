use alloc::string::ToString;
use spin::Lazy;

use crate::{
    drivers::storage::BLOCK_DEVICES,
    fs::{
        ext2::Ext2FileSystem,
        vfs::{file::ArcFile, fs::ArcFileSystem},
    },
};

pub mod ext2;
pub mod vfs;

pub static ROOT_FILE_SYSTEM: Lazy<ArcFileSystem> = Lazy::new(|| {
    for block_device in BLOCK_DEVICES.lock().iter() {
        if let Some(fs) = Ext2FileSystem::new(block_device.clone()) {
            info!("Mount root file system OK");
            return fs;
        }
    }
    panic!("Failed to mount root");
});

pub static ROOT_DIR: Lazy<ArcFile> = Lazy::new(|| {
    ROOT_FILE_SYSTEM
        .write()
        .lookup("/".to_string(), false)
        .expect("Failed to get root dir")
});
