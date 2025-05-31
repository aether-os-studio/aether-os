use linked_list_allocator::LockedHeap;
use rmm::{Arch, PageFlags, VirtualAddress};

use crate::arch::CurrentMMArch;
use crate::arch::rmm::PageMapper;

use super::frame::TheFrameAllocator;

pub const KERNEL_HEAP_START: usize = 0xffff_c000_0000_0000;
pub const KERNEL_HEAP_SIZE: usize = 32 * 1024 * 1024;

#[global_allocator]
pub static KERNEL_ALLOCATOR: LockedHeap = LockedHeap::empty();

pub fn init() {
    let mut mapper = unsafe { PageMapper::current(rmm::TableKind::Kernel, TheFrameAllocator) };

    unsafe {
        for i in 0..(KERNEL_HEAP_SIZE / CurrentMMArch::PAGE_SIZE) {
            mapper
                .map(
                    VirtualAddress::new(KERNEL_HEAP_START + i * CurrentMMArch::PAGE_SIZE),
                    PageFlags::new().write(true),
                )
                .unwrap()
                .flush();
        }

        KERNEL_ALLOCATOR
            .lock()
            .init(KERNEL_HEAP_START as *mut u8, KERNEL_HEAP_SIZE);
    };
}
