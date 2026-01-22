use alloc::{sync::Arc, vec::Vec};
use spin::{Mutex, RwLock};

use crate::utils::Dma;

pub mod usb;

pub trait BlockDeviceTrait: Send + Sync {
    fn read_block(&mut self, block_id: u64, buf: &mut [u8]) -> Result<(), &'static str>;
    fn write_block(&mut self, block_id: u64, buf: &[u8]) -> Result<(), &'static str>;
    fn block_size(&self) -> u32;
    fn block_count(&self) -> u64;
}

pub type ArcBlockDeviceTrait = Arc<RwLock<dyn BlockDeviceTrait>>;

#[derive(Clone)]
pub struct BlockDevice {
    inner: ArcBlockDeviceTrait,
}

impl BlockDevice {
    pub fn new(inner: ArcBlockDeviceTrait) -> Self {
        Self { inner }
    }
}

impl BlockDevice {
    pub fn read_block(&mut self, start_byte: u64, buf: &mut [u8]) -> Result<(), &'static str> {
        if buf.is_empty() {
            return Ok(());
        }

        let block_size = self.block_size() as usize;
        let mut block_dev = self.inner.write();

        let start = start_byte as usize;
        let end = start + buf.len();

        let start_block_id = start / block_size;
        let end_block_id = (end - 1) / block_size;

        let mut temp_block = unsafe {
            Dma::<[u8]>::zeroed_slice(block_size)
                .expect("No memory for block device")
                .cast_slice()
        };
        let mut buf_offset = 0;

        for block_id in start_block_id..=end_block_id {
            block_dev.read_block(block_id as u64, &mut temp_block)?;

            let block_start_byte = block_id * block_size;

            let offset_in_block = if block_id == start_block_id {
                start - block_start_byte
            } else {
                0
            };

            let end_in_block = if block_id == end_block_id {
                end - block_start_byte
            } else {
                block_size
            };

            let bytes_to_copy = end_in_block - offset_in_block;

            buf[buf_offset..buf_offset + bytes_to_copy]
                .copy_from_slice(&temp_block[offset_in_block..end_in_block]);

            buf_offset += bytes_to_copy;
        }

        Ok(())
    }

    pub fn write_block(&mut self, start_byte: u64, buf: &[u8]) -> Result<(), &'static str> {
        if buf.is_empty() {
            return Ok(());
        }

        let block_size = self.block_size() as usize;
        let mut block_dev = self.inner.write();

        let start = start_byte as usize;
        let end = start + buf.len();

        let start_block_id = start / block_size;
        let end_block_id = (end - 1) / block_size;

        let mut temp_block = unsafe {
            Dma::<[u8]>::zeroed_slice(block_size)
                .expect("No memory for block device")
                .cast_slice()
        };
        let mut buf_offset = 0;

        for block_id in start_block_id..=end_block_id {
            let block_start_byte = block_id * block_size;

            let offset_in_block = if block_id == start_block_id {
                start - block_start_byte
            } else {
                0
            };

            let end_in_block = if block_id == end_block_id {
                end - block_start_byte
            } else {
                block_size
            };

            let bytes_to_copy = end_in_block - offset_in_block;

            if offset_in_block != 0 || end_in_block != block_size {
                block_dev.read_block(block_id as u64, &mut temp_block)?;
            }

            temp_block[offset_in_block..end_in_block]
                .copy_from_slice(&buf[buf_offset..buf_offset + bytes_to_copy]);

            block_dev.write_block(block_id as u64, &temp_block)?;

            buf_offset += bytes_to_copy;
        }

        Ok(())
    }

    pub fn block_size(&self) -> u32 {
        self.inner.read().block_size()
    }
    pub fn block_count(&self) -> u64 {
        self.inner.read().block_count()
    }
}

pub static BLOCK_DEVICES: Mutex<Vec<BlockDevice>> = Mutex::new(Vec::new());

pub fn init() {
    usb::init();
}
