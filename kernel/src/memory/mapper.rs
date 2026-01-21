use core::marker::PhantomData;

use crate::memory::{
    DummyFrameAllocator,
    page::{add_ref_count, can_free, sub_ref_count},
};
use rmm::{
    Arch, FrameAllocator, PageFlags, PageFlush, PageMapper, PageTable, PhysicalAddress, TableKind,
    VirtualAddress,
};

pub struct KernelPageMapper<A: Arch, F: FrameAllocator> {
    kind: TableKind,
    addr: PhysicalAddress,
    allocator: F,
    _arch: PhantomData<A>,
}

impl<A: Arch, F: FrameAllocator> KernelPageMapper<A, F> {
    pub unsafe fn new(kind: TableKind, addr: PhysicalAddress, allocator: F) -> Self {
        Self {
            kind,
            addr,
            allocator,
            _arch: PhantomData,
        }
    }

    pub unsafe fn current(table_kind: TableKind, allocator: F) -> Self {
        let table_addr = A::table(table_kind);
        Self::new(table_kind, table_addr, allocator)
    }

    pub unsafe fn create(table_kind: TableKind, mut allocator: F) -> Option<Self> {
        unsafe {
            let table_addr = allocator.allocate_one()?;
            Some(Self::new(table_kind, table_addr, allocator))
        }
    }

    pub fn is_current(&self) -> bool {
        unsafe { self.table().phys() == A::table(self.kind) }
    }

    pub unsafe fn make_current(&self) {
        unsafe {
            A::set_table(self.kind, self.addr);
        }
    }

    pub fn table(&self) -> PageTable<A> {
        // SAFETY: The only way to initialize a PageMapper is via new(), and we assume it upholds
        // all necessary invariants for this to be safe.
        unsafe { PageTable::new(VirtualAddress::new(0), self.addr, A::PAGE_LEVELS - 1) }
    }

    pub unsafe fn map(
        &mut self,
        virt: VirtualAddress,
        flags: PageFlags<A>,
    ) -> Option<PageFlush<A>> {
        unsafe {
            let phys = self.allocator.allocate_one()?;
            add_ref_count(phys);
            self.map_phys(virt, phys, flags)
        }
    }

    pub unsafe fn map_phys(
        &mut self,
        virt: VirtualAddress,
        phys: PhysicalAddress,
        flags: PageFlags<A>,
    ) -> Option<PageFlush<A>> {
        let mut page_mapper =
            unsafe { PageMapper::<A, &mut F>::new(self.kind, self.addr, &mut self.allocator) };
        page_mapper.map_phys(virt, phys, flags)
    }

    pub unsafe fn unmap(
        &mut self,
        virt: VirtualAddress,
        unmap_parents: bool,
    ) -> Option<PageFlush<A>> {
        unsafe {
            let (old, _, flush) = self.unmap_phys(virt, unmap_parents)?;
            sub_ref_count(old);
            if can_free(old) {
                self.allocator.free_one(old);
            }
            Some(flush)
        }
    }

    pub unsafe fn unmap_phys(
        &mut self,
        virt: VirtualAddress,
        unmap_parents: bool,
    ) -> Option<(PhysicalAddress, PageFlags<A>, PageFlush<A>)> {
        let mut page_mapper =
            unsafe { PageMapper::<A, &mut F>::new(self.kind, self.addr, &mut self.allocator) };
        page_mapper.unmap_phys(virt, unmap_parents)
    }

    pub fn translate(&self, virt: VirtualAddress) -> Option<(PhysicalAddress, PageFlags<A>)> {
        let page_mapper = unsafe {
            PageMapper::<A, DummyFrameAllocator>::new(self.kind, self.addr, DummyFrameAllocator)
        };
        page_mapper.translate(virt)
    }
}
