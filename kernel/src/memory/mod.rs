use core::cell::SyncUnsafeCell;
use rmm::PhysicalAddress;

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

pub mod heap;
