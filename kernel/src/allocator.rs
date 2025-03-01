use rmm::{Flusher, PageFlags, PageFlushAll, VirtualAddress};

use crate::{arch::memory::paging::Page, memory::mapper::KernelMapper};

#[global_allocator]
pub static ALLOCATOR: linked_list_allocator::LockedHeap =
    linked_list_allocator::LockedHeap::empty();

unsafe fn map_heap(mapper: &mut KernelMapper, offset: usize, size: usize) {
    let mapper = mapper
        .get_mut()
        .expect("failed to obtain exclusive access to KernelMapper while extending heap");
    let mut flush_all = PageFlushAll::new();

    let heap_start_page = Page::containing_address(VirtualAddress::new(offset));
    let heap_end_page = Page::containing_address(VirtualAddress::new(offset + size - 1));
    for page in Page::range_inclusive(heap_start_page, heap_end_page) {
        let result = mapper
            .map(page.start_address(), PageFlags::new().write(true))
            .expect("failed to map kernel heap");
        flush_all.consume(result);
    }

    flush_all.flush();
}

pub unsafe fn init() {
    let offset = crate::KERNEL_HEAP_OFFSET;
    let size = crate::KERNEL_HEAP_SIZE;

    // Map heap pages
    map_heap(&mut KernelMapper::lock(), offset, size);

    ALLOCATOR.lock().init(offset as *mut u8, size);
}
