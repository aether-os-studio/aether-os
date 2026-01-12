use core::{cell::SyncUnsafeCell, sync::atomic::AtomicUsize};

use crate::arch::{CurrentRmmArch, rmm::page_flags};
use limine::{memory_map::EntryType, request::MemoryMapRequest, response::MemoryMapResponse};
use rmm::{
    Arch, BuddyAllocator, BumpAllocator, MemoryArea, PageMapper, PhysicalAddress, TableKind,
    VirtualAddress,
};
use spin::{Lazy, Mutex};

pub const PAGE_SIZE: usize = CurrentRmmArch::PAGE_SIZE;

#[used]
#[unsafe(link_section = ".requests")]
static MEMMAP_REQUEST: MemoryMapRequest = MemoryMapRequest::new();

unsafe fn add_memory(
    areas: &mut [MemoryArea],
    area_i: &mut usize,
    start: PhysicalAddress,
    length: usize,
) {
    areas[*area_i].base = start;
    areas[*area_i].size = length;
    *area_i += 1;
}

pub fn align_up(x: usize) -> usize {
    (x.saturating_add(PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE
}
pub fn align_down(x: usize) -> usize {
    x / PAGE_SIZE * PAGE_SIZE
}

static NULL_AREAS: SyncUnsafeCell<[rmm::MemoryArea; 1]> = SyncUnsafeCell::new(
    [rmm::MemoryArea {
        base: PhysicalAddress::new(0),
        size: 0,
    }; 1],
);

pub static KERNEL_PAGE_TABLE_PHYS: AtomicUsize = AtomicUsize::new(0);

unsafe fn map_memory<A: Arch>(
    mut bump_allocator: &mut BumpAllocator<A>,
    memmap_response: &MemoryMapResponse,
) {
    let mut null_bump_allocator = BumpAllocator::<A>::new(unsafe { &*NULL_AREAS.get() }, 0);
    let old_mapper = PageMapper::<A, _>::current(TableKind::Kernel, &mut null_bump_allocator);
    let mut mapper = PageMapper::<A, _>::create(TableKind::Kernel, &mut bump_allocator)
        .expect("failed to create Mapper");

    let start = align_down(crate::kernel_executable_offsets::__start());
    let end = align_up(crate::kernel_executable_offsets::__end());

    for addr in (start..end).step_by(PAGE_SIZE) {
        let virt = VirtualAddress::new(addr);
        let phys = old_mapper.translate(virt).unwrap().0;
        let flags = page_flags::<A>(virt);
        mapper.map_phys(virt, phys, flags).unwrap().ignore();
    }

    for entry in memmap_response.entries().iter() {
        if entry.entry_type == EntryType::USABLE
            || entry.entry_type == EntryType::FRAMEBUFFER
            || entry.entry_type == EntryType::EXECUTABLE_AND_MODULES
            || entry.entry_type == EntryType::BOOTLOADER_RECLAIMABLE
        {
            let start = align_down(entry.base as usize);
            let end = align_up(start + entry.length as usize);

            for addr in (start..end).step_by(PAGE_SIZE) {
                let phys = PhysicalAddress::new(addr);
                let virt = CurrentRmmArch::phys_to_virt(phys);
                let flags = page_flags::<A>(virt);
                mapper.map_phys(virt, phys, flags).unwrap().ignore();
            }
        }
    }

    mapper.make_current();

    let phys = mapper.table().phys();
    KERNEL_PAGE_TABLE_PHYS.store(phys.data(), core::sync::atomic::Ordering::SeqCst);
}

pub static FRAME_ALLOCATOR: Lazy<Mutex<BuddyAllocator<CurrentRmmArch>>> = Lazy::new(|| {
    let memmap_response = MEMMAP_REQUEST.get_response().unwrap();

    let areas = &mut unsafe { *crate::memory::AREAS.get() };
    let mut area_i = 0;

    for area in memmap_response.entries().iter() {
        if area.entry_type == EntryType::USABLE {
            unsafe {
                add_memory(
                    areas,
                    &mut area_i,
                    PhysicalAddress::new(area.base as usize),
                    area.length as usize,
                );
            }
        }
    }

    areas[..area_i].sort_unstable_by_key(|area| area.base);
    unsafe { crate::memory::AREAS.get().write(*areas) };
    unsafe { crate::memory::AREA_COUNT.get().write(area_i as u16) };

    let areas = crate::memory::areas();
    let mut bump_allocator = BumpAllocator::<CurrentRmmArch>::new(areas, 0);

    unsafe { map_memory(&mut bump_allocator, memmap_response) };

    let buddy_allocator =
        unsafe { BuddyAllocator::new(bump_allocator) }.expect("Failed to init mm");

    Mutex::new(buddy_allocator)
});
