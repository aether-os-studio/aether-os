use alloc::{boxed::Box, vec::Vec};
use x86_64::{VirtAddr, structures::paging::OffsetPageTable};

use crate::memory::{MappingType, MemoryManager};

pub const STACK_SIZE: usize = 64 * 1024;

pub struct KernelStack(Box<Vec<u8>>);

impl KernelStack {
    pub fn new() -> Self {
        Self(Box::new(alloc::vec![0u8; STACK_SIZE]))
    }

    pub fn stack_top(&self) -> VirtAddr {
        VirtAddr::new(self.0.as_ptr_range().end as u64)
    }
}

const USER_STACK_END: usize = 0x7fffffff0000;
const USER_STACK_SIZE: usize = 256 * 1024;

pub struct UserStack;

impl UserStack {
    pub fn end_address() -> VirtAddr {
        VirtAddr::new(USER_STACK_END as u64)
    }
}

impl UserStack {
    pub fn map(page_table: &mut OffsetPageTable<'static>) {
        let end_address = VirtAddr::new(USER_STACK_END as u64);

        MemoryManager::alloc_range(
            end_address - USER_STACK_SIZE as u64,
            USER_STACK_SIZE as u64,
            MappingType::UserData.flags(),
            page_table,
        )
        .unwrap();
    }
}
