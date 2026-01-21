use core::cell::SyncUnsafeCell;
use rmm::{FrameAllocator, FrameCount, FrameUsage, PhysicalAddress};

pub(crate) static AREAS: SyncUnsafeCell<[rmm::MemoryArea; 1024]> = SyncUnsafeCell::new(
    [rmm::MemoryArea {
        base: PhysicalAddress::new(0),
        size: 0,
    }; 1024],
);
pub(crate) static AREA_COUNT: SyncUnsafeCell<u16> = SyncUnsafeCell::new(0);

pub(crate) fn areas() -> &'static [rmm::MemoryArea] {
    unsafe { &(&*AREAS.get())[..AREA_COUNT.get().read().into()] }
}

pub struct DummyFrameAllocator;

impl FrameAllocator for DummyFrameAllocator {
    unsafe fn allocate(&mut self, _count: FrameCount) -> Option<PhysicalAddress> {
        None
    }

    unsafe fn free(&mut self, _address: PhysicalAddress, _count: FrameCount) {}

    unsafe fn usage(&self) -> FrameUsage {
        FrameUsage::new(FrameCount::new(0), FrameCount::new(0))
    }
}

pub mod heap;
