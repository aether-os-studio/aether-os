use core::sync::atomic::AtomicUsize;

use rmm::PhysicalAddress;
use spin::Mutex;

use crate::init::memory::PAGE_SIZE;

pub struct Page {
    ref_count: AtomicUsize,
}

impl Page {
    pub fn add_ref_count(&self) {
        self.ref_count
            .fetch_add(1, core::sync::atomic::Ordering::SeqCst);
    }

    pub fn sub_ref_count(&self) {
        self.ref_count
            .fetch_sub(1, core::sync::atomic::Ordering::SeqCst);
    }

    pub fn can_free(&self) -> bool {
        self.ref_count.load(core::sync::atomic::Ordering::SeqCst) == 0
    }
}

pub struct PageManager {
    pages: *mut Page,
    count: usize,
}

impl PageManager {
    pub fn new(pages: *mut Page, count: usize) -> Self {
        Self { pages, count }
    }

    fn pages(&self) -> &mut [Page] {
        unsafe { core::slice::from_raw_parts_mut(self.pages, self.count) }
    }

    pub fn add_ref_count(&self, addr: PhysicalAddress) {
        let pages = self.pages();
        pages[addr.data().div_floor(PAGE_SIZE)].add_ref_count();
    }

    pub fn sub_ref_count(&self, addr: PhysicalAddress) {
        let pages = self.pages();
        pages[addr.data().div_floor(PAGE_SIZE)].sub_ref_count();
    }

    pub fn can_free(&self, addr: PhysicalAddress) -> bool {
        let pages = self.pages();
        pages[addr.data().div_floor(PAGE_SIZE)].can_free()
    }
}

unsafe impl Send for PageManager {}
unsafe impl Sync for PageManager {}

pub static PAGES: Mutex<Option<PageManager>> = Mutex::new(None);

pub fn add_ref_count(addr: PhysicalAddress) {
    if let Some(pages) = PAGES.lock().as_mut() {
        pages.add_ref_count(addr);
    }
}
pub fn sub_ref_count(addr: PhysicalAddress) {
    if let Some(pages) = PAGES.lock().as_mut() {
        pages.sub_ref_count(addr);
    }
}
pub fn can_free(addr: PhysicalAddress) -> bool {
    if let Some(pages) = PAGES.lock().as_mut() {
        pages.can_free(addr)
    } else {
        false
    }
}
