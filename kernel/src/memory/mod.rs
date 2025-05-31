use core::cell::SyncUnsafeCell;

use limine::{memory_map::EntryType, request::MemoryMapRequest};
use rmm::{BuddyAllocator, BumpAllocator, MemoryArea, PhysicalAddress};
use spin::{Lazy, Mutex};

use crate::arch::CurrentMMArch;

#[used]
#[unsafe(link_section = ".requests")]
static MEMMAP_REQUEST: MemoryMapRequest = MemoryMapRequest::new();

pub(crate) static AREAS: SyncUnsafeCell<[rmm::MemoryArea; 512]> = SyncUnsafeCell::new(
    [rmm::MemoryArea {
        base: PhysicalAddress::new(0),
        size: 0,
    }; 512],
);
pub(crate) static AREA_COUNT: SyncUnsafeCell<u16> = SyncUnsafeCell::new(0);

pub static FRAME_ALLOCATOR: Lazy<Mutex<BuddyAllocator<CurrentMMArch>>> = Lazy::new(|| {
    let usable_areas = MEMMAP_REQUEST
        .get_response()
        .unwrap()
        .entries()
        .iter()
        .filter(|r| r.entry_type == EntryType::USABLE)
        .map(|r| MemoryArea {
            base: PhysicalAddress::new(r.base as usize),
            size: r.length as usize,
        });

    let areas = unsafe { &mut *AREAS.get() };
    let mut area_i = 0;

    for area in usable_areas {
        areas[area_i] = area;
        area_i += 1;
    }

    areas[..area_i].sort_unstable_by_key(|area| area.base);
    unsafe { AREA_COUNT.get().write(area_i as u16) };

    let bump_allocator = BumpAllocator::<CurrentMMArch>::new(areas, 0);

    let buddy_allocator = unsafe { BuddyAllocator::new(bump_allocator) }.unwrap();

    Mutex::new(buddy_allocator)
});

pub fn init() {
    Lazy::force(&FRAME_ALLOCATOR);
}

pub mod frame;
pub mod heap;
