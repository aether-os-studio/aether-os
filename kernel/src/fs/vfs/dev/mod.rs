use alloc::{format, string::ToString, sync::Arc};
use spin::{Mutex, RwLock};

use crate::driver::block::HD_LIST;

use super::{
    ROOT,
    fake::FakeFS,
    inode::{InodeRef, mount_to},
};

pub mod block;
pub mod gpt_parser;
pub mod partition;

pub static ROOT_PARTITION: Mutex<Option<InodeRef>> = Mutex::new(None);

fn provide_hard_disk(hd: usize, dev_fs: InodeRef) {
    let id_to_alpha = [
        "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r",
        "s", "t", "u", "v", "w", "x", "y", "z",
    ];

    let block_i = Arc::new(RwLock::new(block::BlockInode::new(hd)));
    mount_to(
        block_i.clone(),
        dev_fs.clone(),
        format!("hd{}", id_to_alpha[hd]),
    );

    let _ = gpt_parser::parse_gpt_disk(hd, block_i.clone(), dev_fs.clone());
}

fn provide_hard_disks(dev_fs: InodeRef) {
    let hd_num = HD_LIST.lock().len();
    for hd in 0..hd_num {
        provide_hard_disk(hd, dev_fs.clone());
    }
}

pub fn init() {
    let dev_fs = FakeFS::new();
    mount_to(dev_fs.clone(), ROOT.lock().clone(), "dev".to_string());

    provide_hard_disks(dev_fs.clone());
}
