use good_memory_allocator::SpinLockedAllocator;
use rmm::{Arch, PageFlags, VirtualAddress};

use crate::arch::CurrentMMArch;

use super::KERNEL_PAGE_TABLE;

pub const KERNEL_HEAP_START: usize = 0xffff_c000_0000_0000;
pub const KERNEL_HEAP_SIZE: usize = 64 * 1024 * 1024;

#[global_allocator]
pub static KERNEL_ALLOCATOR: SpinLockedAllocator = SpinLockedAllocator::empty();

pub fn init() {
    unsafe {
        for i in 0..(KERNEL_HEAP_SIZE / CurrentMMArch::PAGE_SIZE) {
            KERNEL_PAGE_TABLE
                .lock()
                .map(
                    VirtualAddress::new(KERNEL_HEAP_START + i * CurrentMMArch::PAGE_SIZE),
                    PageFlags::new().write(true),
                )
                .unwrap()
                .flush();
        }

        KERNEL_ALLOCATOR.init(KERNEL_HEAP_START, KERNEL_HEAP_SIZE);
    };
}
