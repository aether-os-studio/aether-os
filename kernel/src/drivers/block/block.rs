use alloc::{boxed::Box, vec::Vec};
use spin::Mutex;

use crate::drivers::base::block::BlockDeviceBase;

use super::nvme::NVME;

pub static BLOCK_DEVICES: Mutex<Vec<Box<dyn BlockDeviceBase>>> = Mutex::new(Vec::new());

pub fn init() {
    for nvme in NVME.iter() {
        for device in nvme {
            BLOCK_DEVICES.lock().push(Box::new(device));
        }
    }
}
