use frame::TheFrameAllocator;
use limine::request::MemoryMapRequest;
use spin::{Lazy, Mutex};

use crate::{arch::rmm::PageMapper, memory::frame::BitmapFrameAllocator};

#[used]
#[unsafe(link_section = ".requests")]
static MEMMAP_REQUEST: MemoryMapRequest = MemoryMapRequest::new();

pub static FRAME_ALLOCATOR: Lazy<Mutex<BitmapFrameAllocator>> = Lazy::new(|| {
    Mutex::new(BitmapFrameAllocator::init(
        MEMMAP_REQUEST.get_response().unwrap(),
    ))
});

pub static KERNEL_PAGE_TABLE: Lazy<Mutex<PageMapper>> = Lazy::new(|| {
    Mutex::new(unsafe { PageMapper::current(rmm::TableKind::Kernel, TheFrameAllocator) })
});

pub fn init() {
    Lazy::force(&FRAME_ALLOCATOR);
}

pub mod bitmap;
pub mod frame;
pub mod heap;
