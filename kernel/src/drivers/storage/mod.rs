use alloc::{sync::Arc, vec::Vec};
use spin::{Mutex, RwLock};

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
        let block_size = self.block_size() as usize;
        let mut block_dev = self.inner.write();

        let start = start_byte as usize;
        let end = start + buf.len();

        let start_sector_read_start = start % block_size;

        let start_sector_id = start / block_size;
        let end_sector_id = (end - 1) / block_size;

        let buffer_size = (end_sector_id - start_sector_id + 1) * block_size;

        let mut buffer = vec![0; buffer_size];
        block_dev.read_block(start_sector_id as u64, &mut buffer)?;
        buf.copy_from_slice(unsafe {
            core::slice::from_raw_parts(
                buffer.as_ptr().byte_add(start_sector_read_start),
                buf.len(),
            )
        });
        Ok(())
    }

    pub fn write_block(&mut self, start_byte: u64, buf: &[u8]) -> Result<(), &'static str> {
        let block_size = self.block_size() as usize;
        let mut block_dev = self.inner.write();

        let start = start_byte as usize;
        let end = start + buf.len();

        let start_sector_write_start = start % block_size;

        let start_sector_id = start / block_size;
        let end_sector_id = (end - 1) / block_size;

        let buffer_size = (end_sector_id - start_sector_id + 1) * block_size;

        let mut buffer = vec![0; buffer_size];
        block_dev.read_block(start_sector_id as u64, &mut buffer)?;
        let buf_in_buffer = unsafe {
            core::slice::from_raw_parts_mut(
                buffer.as_mut_ptr().byte_add(start_sector_write_start),
                buf.len(),
            )
        };
        buf_in_buffer.copy_from_slice(buf);
        block_dev.write_block(start_sector_id as u64, &buffer)
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
