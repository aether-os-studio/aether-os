use alloc::vec::Vec;
use gpt_disk_io::{
    BlockIo, Disk, DiskError,
    gpt_disk_types::{BlockSize, GptPartitionEntryArrayLayout, GptPartitionEntrySize, Lba},
};
use rmm::VirtualAddress;
use spin::Mutex;

use crate::{drivers::base::block::BlockDeviceBase, errno::Errno};

use super::{block::BLOCK_DEVICES, cache::CachedBlockDevice};

impl BlockIo for CachedBlockDevice {
    type Error = usize;

    fn num_blocks(&mut self) -> Result<u64, Self::Error> {
        Ok(self.inner.block_count() as u64)
    }

    fn block_size(&self) -> BlockSize {
        BlockSize::new(self.inner.block_size() as u32).unwrap()
    }

    fn read_blocks(&mut self, start_lba: Lba, dst: &mut [u8]) -> Result<(), Self::Error> {
        self.read(
            start_lba.0 as usize * self.inner.block_size(),
            dst.len(),
            VirtualAddress::new(dst.as_mut_ptr() as usize),
        );
        Ok(())
    }

    fn write_blocks(&mut self, start_lba: Lba, src: &[u8]) -> Result<(), Self::Error> {
        self.write(
            start_lba.0 as usize * self.inner.block_size(),
            src.len(),
            VirtualAddress::new(src.as_ptr() as usize),
        );
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_all();
        Ok(())
    }
}

#[derive(Clone)]
pub struct PartitionDevice {
    device_id: usize,
    start_bytes: usize,
    total_bytes: usize,
}

impl PartitionDevice {
    pub fn read_at(&self, offset: usize, buf: &mut [u8]) -> Result<usize, Errno> {
        let mut cached_block_device = CachedBlockDevice::new(self.device_id);
        let len = cached_block_device.read(
            offset + self.start_bytes,
            buf.len(),
            VirtualAddress::new(buf.as_mut_ptr() as usize),
        );
        Ok(len)
    }

    pub fn write_at(&self, offset: usize, buf: &[u8]) -> Result<usize, Errno> {
        let mut cached_block_device = CachedBlockDevice::new(self.device_id);
        let len = cached_block_device.read(
            offset + self.start_bytes,
            buf.len(),
            VirtualAddress::new(buf.as_ptr() as usize),
        );
        Ok(len)
    }
}

pub static PARTITION_DEVICES: Mutex<Vec<PartitionDevice>> = Mutex::new(Vec::new());

pub fn init() -> Result<(), DiskError<usize>> {
    for (i, block_device) in BLOCK_DEVICES.lock().iter().enumerate() {
        let device = CachedBlockDevice::new(i);

        let mut gpt = Disk::new(device)?;

        let mut buf = Vec::new();
        for _ in 0..block_device.block_size() {
            buf.push(0);
        }

        unsafe { BLOCK_DEVICES.force_unlock() };

        let header = gpt.read_gpt_header(Lba(1), &mut buf)?;

        if let Some(entry_size) =
            GptPartitionEntrySize::new(header.size_of_partition_entry.to_u32()).ok()
        {
            let part_iter = gpt.gpt_partition_entry_array_iter(
                GptPartitionEntryArrayLayout {
                    start_lba: header.partition_entry_lba.into(),
                    entry_size,
                    num_entries: header.number_of_partition_entries.to_u32(),
                },
                &mut buf,
            )?;

            let id_to_alpha = [
                "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p",
                "q", "r", "s", "t", "u", "v", "w", "x", "y", "z",
            ];

            for (partition_id, part) in part_iter.enumerate() {
                if let Ok(part) = part {
                    if part.is_used() {
                        let device = PartitionDevice {
                            device_id: i,
                            start_bytes: part.starting_lba.to_u64() as usize
                                * block_device.block_size(),
                            total_bytes: (part.ending_lba.to_u64() as usize
                                - part.starting_lba.to_u64() as usize
                                + 1)
                                * block_device.block_size(),
                        };

                        PARTITION_DEVICES.lock().push(device);
                    }
                }
            }
        } else {
            let device = PartitionDevice {
                device_id: i,
                start_bytes: 0,
                total_bytes: block_device.block_count() * block_device.block_size(),
            };

            PARTITION_DEVICES.lock().push(device);
        }
    }

    Ok(())
}
