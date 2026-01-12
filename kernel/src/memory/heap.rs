use crate::{arch::CurrentRmmArch, init::memory::PAGE_SIZE};
use good_memory_allocator::SpinLockedAllocator;
use rmm::{PageFlags, PageMapper, VirtualAddress};

use crate::init::memory::FRAME_ALLOCATOR;

#[global_allocator]
pub static KERNEL_ALLOCATOR: SpinLockedAllocator = SpinLockedAllocator::empty();

pub const KERNEL_HEAP_START: usize = 0xffffffff_c0000000;
pub const KERNEL_HEAP_SIZE: usize = 64 * 1024 * 1024;

pub fn init() {
    let mut frame_allocator = FRAME_ALLOCATOR.lock();
    let mut mapper = unsafe { PageMapper::current(rmm::TableKind::Kernel, &mut *frame_allocator) };

    for addr in (KERNEL_HEAP_START..(KERNEL_HEAP_START + KERNEL_HEAP_SIZE)).step_by(PAGE_SIZE) {
        let virt = VirtualAddress::new(addr);
        let flags = PageFlags::<CurrentRmmArch>::new().write(true);
        unsafe { mapper.map(virt, flags).unwrap().flush() };
    }

    unsafe { KERNEL_ALLOCATOR.init(KERNEL_HEAP_START, KERNEL_HEAP_SIZE) };
}
