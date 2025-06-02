use limine::memory_map::EntryType;
use limine::response::MemoryMapResponse;
use rmm::{Arch, FrameAllocator, FrameCount, FrameUsage, PhysicalAddress};

use crate::arch::CurrentMMArch;
use crate::memory::bitmap::Bitmap;

use super::FRAME_ALLOCATOR;

pub struct BitmapFrameAllocator {
    bitmap: Bitmap,
    origin_frames: usize,
    usable_frames: usize,
    last_search_idx: usize,
}

impl BitmapFrameAllocator {
    pub fn init(memory_map: &MemoryMapResponse) -> Self {
        let memory_size = memory_map
            .entries()
            .last()
            .map(|region| region.base + region.length)
            .expect("No memory regions found");

        let bitmap_size = (memory_size / CurrentMMArch::PAGE_SIZE as u64).div_ceil(8) as usize;

        let usable_regions = memory_map
            .entries()
            .iter()
            .filter(|region| region.entry_type == EntryType::USABLE);

        let bitmap_address = usable_regions
            .clone()
            .find(|region| region.length >= bitmap_size as u64)
            .map(|region| region.base)
            .expect("No suitable memory region for bitmap");

        let bitmap_buffer = unsafe {
            let physical_address = PhysicalAddress::new(bitmap_address as usize);
            let virtual_address = { CurrentMMArch::phys_to_virt(physical_address) };
            let bitmap_inner_size = bitmap_size / size_of::<usize>();
            core::slice::from_raw_parts_mut(virtual_address.data() as *mut usize, bitmap_inner_size)
        };

        let mut bitmap = Bitmap::new(bitmap_buffer);
        let mut origin_frames = 0;

        for region in usable_regions {
            let start_page_index = (region.base / CurrentMMArch::PAGE_SIZE as u64) as usize;
            let frame_count = (region.length / CurrentMMArch::PAGE_SIZE as u64) as usize;

            origin_frames += frame_count;
            bitmap.set_range(start_page_index, start_page_index + frame_count, true);
        }

        let bitmap_frame_start = (bitmap_address / CurrentMMArch::PAGE_SIZE as u64) as usize;
        let bitmap_frame_count =
            (bitmap_size + CurrentMMArch::PAGE_SIZE - 1).div_ceil(CurrentMMArch::PAGE_SIZE);
        let bitmap_frame_end = bitmap_frame_start + bitmap_frame_count;

        let usable_frames = origin_frames - bitmap_frame_count;
        bitmap.set_range(bitmap_frame_start, bitmap_frame_end, false);

        BitmapFrameAllocator {
            bitmap,
            origin_frames,
            usable_frames,
            last_search_idx: 0,
        }
    }
}

impl FrameAllocator for BitmapFrameAllocator {
    unsafe fn allocate(&mut self, count: FrameCount) -> Option<PhysicalAddress> {
        let index = self
            .bitmap
            .find_range_from(count.data(), true, self.last_search_idx)
            .or_else(|| {
                self.last_search_idx = 0;
                self.bitmap.find_range_from(count.data(), true, 0)
            })?;

        self.bitmap.set_range(index, index + count.data(), false);

        Some(PhysicalAddress::new(index * CurrentMMArch::PAGE_SIZE))
    }

    unsafe fn free(&mut self, address: PhysicalAddress, count: FrameCount) {
        let index = address.data() / CurrentMMArch::PAGE_SIZE;
        self.bitmap
            .set_range(index as usize, index + count.data(), true);
        self.usable_frames += count.data();
    }

    unsafe fn usage(&self) -> FrameUsage {
        FrameUsage::new(
            FrameCount::new(self.origin_frames - self.origin_frames),
            FrameCount::new(self.origin_frames),
        )
    }
}

pub struct TheFrameAllocator;

impl FrameAllocator for TheFrameAllocator {
    unsafe fn allocate(&mut self, count: rmm::FrameCount) -> Option<rmm::PhysicalAddress> {
        FRAME_ALLOCATOR.lock().allocate(count)
    }

    unsafe fn free(&mut self, address: rmm::PhysicalAddress, count: rmm::FrameCount) {
        FRAME_ALLOCATOR.lock().free(address, count);
    }

    unsafe fn usage(&self) -> rmm::FrameUsage {
        FRAME_ALLOCATOR.lock().usage()
    }
}
