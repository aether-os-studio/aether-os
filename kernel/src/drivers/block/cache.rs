use core::alloc::Layout;

use rmm::Arch;

use crate::arch::CurrentMMArch;
use crate::drivers::base::block::BlockDeviceBase;

use super::block::BLOCK_DEVICES;

pub struct NoCachedBlockDevice {
    id: usize,
}

impl NoCachedBlockDevice {
    pub fn new(id: usize) -> NoCachedBlockDevice {
        NoCachedBlockDevice { id }
    }
}

impl BlockDeviceBase for NoCachedBlockDevice {
    fn block_count(&self) -> usize {
        if let Some(device) = BLOCK_DEVICES.lock().get(self.id) {
            return device.block_count();
        }
        0
    }

    fn block_size(&self) -> usize {
        if let Some(device) = BLOCK_DEVICES.lock().get(self.id) {
            return device.block_size();
        }
        0
    }

    fn read(&mut self, lba: usize, count: usize, addr: rmm::VirtualAddress) -> usize {
        if let Some(device) = BLOCK_DEVICES.lock().get_mut(self.id) {
            return device.read(lba, count, addr);
        }
        0
    }

    fn write(&mut self, lba: usize, count: usize, addr: rmm::VirtualAddress) -> usize {
        if let Some(device) = BLOCK_DEVICES.lock().get_mut(self.id) {
            return device.write(lba, count, addr);
        }
        0
    }
}

pub struct CachedBlockDevice {
    pub inner: NoCachedBlockDevice,
}

impl CachedBlockDevice {
    pub fn new(id: usize) -> CachedBlockDevice {
        CachedBlockDevice {
            inner: NoCachedBlockDevice::new(id),
        }
    }
}

impl CachedBlockDevice {
    pub fn read(&mut self, offset: usize, size: usize, addr: rmm::VirtualAddress) -> usize {
        let block_size = self.inner.block_size();
        let total_blocks = self.inner.block_count();

        let start_block = offset / block_size;
        let block_offset = offset % block_size;

        if start_block >= total_blocks {
            return 0;
        }

        let layout =
            unsafe { Layout::from_size_align_unchecked(block_size, CurrentMMArch::PAGE_SIZE) };

        let temp_buf_ptr = unsafe { alloc::alloc::alloc(layout) };
        let temp_buf = unsafe { core::slice::from_raw_parts_mut(temp_buf_ptr, block_size) };
        let temp_addr = rmm::VirtualAddress::new(temp_buf.as_mut_ptr() as usize);

        let mut bytes_read = 0;
        let mut remaining = size;

        while remaining > 0 && start_block + (bytes_read / block_size) < total_blocks {
            let current_block = start_block + (bytes_read / block_size);
            let read_size = block_size.min(remaining + block_offset);

            let blocks_read = self.inner.read(current_block, 1, temp_addr);
            if blocks_read == 0 {
                break;
            }

            let copy_start = if bytes_read == 0 { block_offset } else { 0 };
            let copy_len = (read_size - copy_start).min(remaining);

            unsafe {
                let dst = addr.add(bytes_read).data() as *mut u8;
                let src = temp_buf[copy_start..copy_start + copy_len].as_ptr();
                core::ptr::copy_nonoverlapping(src, dst, copy_len);
            }

            bytes_read += copy_len;
            remaining -= copy_len;
        }

        unsafe { alloc::alloc::dealloc(temp_buf_ptr, layout) };

        bytes_read
    }

    pub fn write(&mut self, offset: usize, size: usize, addr: rmm::VirtualAddress) -> usize {
        let block_size = self.inner.block_size();
        let total_blocks = self.inner.block_count();

        let start_block = offset / block_size;
        let block_offset = offset % block_size;
        let end_block = (offset + size) / block_size;

        if start_block >= total_blocks {
            return 0;
        }

        let layout =
            unsafe { Layout::from_size_align_unchecked(block_size, CurrentMMArch::PAGE_SIZE) };

        let temp_buf_ptr = unsafe { alloc::alloc::alloc(layout) };
        let temp_buf = unsafe { core::slice::from_raw_parts_mut(temp_buf_ptr, block_size) };
        let mut bytes_written = 0;

        if block_offset != 0 {
            let current_block = start_block;
            let copy_len = (block_size - block_offset).min(size);

            let read_addr = rmm::VirtualAddress::new(temp_buf.as_mut_ptr() as usize);
            self.inner.read(current_block, 1, read_addr);

            unsafe {
                let src = addr.data() as *const u8;
                core::ptr::copy_nonoverlapping(
                    src,
                    temp_buf[block_offset..block_offset + copy_len].as_mut_ptr(),
                    copy_len,
                );
            }

            let write_addr = rmm::VirtualAddress::new(temp_buf.as_ptr() as usize);
            if self.inner.write(current_block, 1, write_addr) == 0 {
                return 0;
            }

            bytes_written += copy_len;
        }

        let full_blocks = (size - bytes_written) / block_size;
        for i in 0..full_blocks {
            let current_block = start_block + 1 + i;
            if current_block >= total_blocks {
                break;
            }

            let block_addr = addr.add(bytes_written + i * block_size);
            if self.inner.write(current_block, 1, block_addr) == 0 {
                break;
            }
            bytes_written += block_size;
        }

        let remaining = size - bytes_written;
        if remaining > 0 {
            let current_block = start_block + 1 + full_blocks;
            if current_block < total_blocks {
                let read_addr = rmm::VirtualAddress::new(temp_buf.as_mut_ptr() as usize);
                self.inner.read(current_block, 1, read_addr);

                let copy_len = remaining.min(block_size);
                unsafe {
                    let src = addr.add(bytes_written).data() as *const u8;
                    core::ptr::copy_nonoverlapping(
                        src,
                        temp_buf[0..copy_len].as_mut_ptr(),
                        copy_len,
                    );
                }

                let write_addr = rmm::VirtualAddress::new(temp_buf.as_ptr() as usize);
                if self.inner.write(current_block, 1, write_addr) > 0 {
                    bytes_written += copy_len;
                }
            }
        }

        unsafe { alloc::alloc::dealloc(temp_buf_ptr, layout) };

        bytes_written
    }

    pub fn flush_all(&mut self) {
        // TODO
    }
}
