use rmm::FrameAllocator;

use super::FRAME_ALLOCATOR;

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
