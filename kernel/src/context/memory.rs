use core::{num::NonZeroUsize, sync::atomic::AtomicU32};

use alloc::sync::Arc;
use rmm::{PageFlags, PhysicalAddress, VirtualAddress};
use syscall::{Error, MapFlags, Result, EFAULT, EINVAL, ENOMEM};

use crate::{
    arch::memory::paging::{PageMapper, PAGE_SIZE},
    cpu_set::LogicalCpuSet,
    memory::RmmA,
};

use super::arch::setup_new_utable;

pub const MMAP_MIN_DEFAULT: usize = PAGE_SIZE;

pub fn page_flags(flags: MapFlags) -> PageFlags<RmmA> {
    PageFlags::new()
        .user(true)
        .execute(flags.contains(MapFlags::PROT_EXEC))
        .write(flags.contains(MapFlags::PROT_WRITE))
    //TODO: PROT_READ
}

pub fn map_flags(page_flags: PageFlags<RmmA>) -> MapFlags {
    let mut flags = MapFlags::PROT_READ;
    if page_flags.has_write() {
        flags |= MapFlags::PROT_WRITE;
    }
    if page_flags.has_execute() {
        flags |= MapFlags::PROT_EXEC;
    }
    flags
}

pub const DANGLING: usize = 1 << (usize::BITS - 2);

#[derive(Debug)]
pub struct Table {
    pub utable: PageMapper,
}

pub struct AddrSpaceWrapper {
    table: Table,
    pub tlb_ack: AtomicU32,
    pub used_by: LogicalCpuSet,
    /// Lowest offset for mmap invocations where the user has not already specified the offset
    /// (using MAP_FIXED/MAP_FIXED_NOREPLACE). Cf. Linux's `/proc/sys/vm/mmap_min_addr`, but with
    /// the exception that we have a memory safe kernel which doesn't have to protect itself
    /// against null pointers, so fixed mmaps to address zero are still allowed.
    pub mmap_min: usize,
}

impl AddrSpaceWrapper {
    pub fn new() -> Result<Arc<Self>> {
        Arc::try_new(Self {
            table: setup_new_utable()?,
            mmap_min: MMAP_MIN_DEFAULT,
            used_by: LogicalCpuSet::empty(),

            tlb_ack: AtomicU32::new(0),
        })
        .map_err(|_| Error::new(ENOMEM))
    }

    pub fn get_phys(&self) -> PhysicalAddress {
        self.table.utable.table().phys()
    }

    pub fn mmap(
        &mut self,
        start_vaddr: VirtualAddress,
        start_paddr: PhysicalAddress,
        page_count: NonZeroUsize,
        map_flags: MapFlags,
    ) -> Result<usize> {
        if start_vaddr.data() < self.mmap_min && map_flags.contains(MapFlags::MAP_FIXED) {
            log::error!(
                "mmap: MAP_FIXED is not supported for address below {}",
                self.mmap_min
            );
            return Err(Error::new(EINVAL));
        }

        let utable = &mut self.table.utable;

        for i in 0..page_count.get() {
            unsafe {
                utable.map_phys(
                    start_vaddr.add(i * PAGE_SIZE),
                    start_paddr.add(i * PAGE_SIZE),
                    page_flags(map_flags),
                )
            }
            .map(|f| {
                f.flush();
                return 0;
            })
            .ok_or(Error::new(EFAULT))?;
        }

        Ok(0)
    }

    pub fn munmap(&mut self, start_vaddr: VirtualAddress) -> Result<usize> {
        let utable = &mut self.table.utable;
        unsafe { utable.unmap_phys(start_vaddr, true) }
            .map(|f| {
                f.2.flush();
                return 0;
            })
            .ok_or(Error::new(EFAULT))
    }
}
