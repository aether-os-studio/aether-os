use alloc::boxed::Box;
use rmm::{PageFlags, VirtualAddress};

use crate::arch::memory::paging::PAGE_SIZE;
use crate::arch::STACK_SIZE;
use crate::memory::allocate_frame;
use crate::memory::mapper::KernelMapper;

const KERNEL_STACK_SIZE: usize = 64 * 1024;
const USER_STACK_END: usize = 0x7fffffff0000;
const USER_STACK_SIZE: usize = STACK_SIZE;

pub struct KernelStack(Box<[u8]>);

impl Default for KernelStack {
    fn default() -> Self {
        Self(Box::from(alloc::vec![0; KERNEL_STACK_SIZE]))
    }
}

impl KernelStack {
    pub fn end_address(&self) -> VirtualAddress {
        VirtualAddress::new(self.0.as_ptr_range().end as usize)
    }
}

pub struct UserStack;

impl UserStack {
    pub fn end_address() -> VirtualAddress {
        VirtualAddress::new(USER_STACK_END)
    }
}

impl UserStack {
    pub fn map(page_table: &mut KernelMapper) -> Option<()> {
        const USER_STACK_START: usize = USER_STACK_END - USER_STACK_SIZE;
        for i in 0..USER_STACK_SIZE / PAGE_SIZE {
            let virt = VirtualAddress::new(USER_STACK_START + i * PAGE_SIZE);
            let phys = allocate_frame().unwrap().base();

            unsafe {
                page_table
                    .get_mut()
                    .unwrap()
                    .map_phys(virt, phys, PageFlags::new().user(true))
            }
            .map(|f| f.flush())?;
        }

        Some(())
    }
}
