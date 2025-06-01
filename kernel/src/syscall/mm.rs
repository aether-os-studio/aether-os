use rmm::{Arch, FrameAllocator, FrameCount, PageFlags, VirtualAddress};

use super::Result;

use crate::arch::CurrentMMArch;
use crate::fs::fd::get_file_descriptor_manager;
use crate::memory::FRAME_ALLOCATOR;
use crate::memory::frame::TheFrameAllocator;
use crate::{errno::Errno, proc::sched::get_current_context};

pub fn sys_brk(addr: usize) -> Result<usize> {
    get_current_context().write().brk(addr)
}

pub const MAP_SHARED: usize = 0x01;
pub const MAP_PRIVATE: usize = 0x02;
pub const MAP_FIXED: usize = 0x10;
pub const MAP_ANONYMOUS: usize = 0x20;

pub const PROT_READ: usize = 0x1;
pub const PROT_WRITE: usize = 0x2;
pub const PROT_EXEC: usize = 0x4;

pub fn sys_mmap(
    addr: usize,
    size: usize,
    prot: usize,
    flags: usize,
    fd: usize,
    offset: usize,
) -> Result<usize> {
    if size == 0 {
        return Err(Errno::ENOMEM);
    }
    if (flags & MAP_FIXED) != 0 && (addr % CurrentMMArch::PAGE_SIZE) != 0 {
        return Err(Errno::EINVAL);
    }

    let ctx = get_current_context();
    let mut ctx_lock = ctx.write();

    let virt_addr = if (flags & MAP_FIXED) != 0 {
        VirtualAddress::new(addr)
    } else {
        ctx_lock.find_free_area(size)
    };

    let frame_count = (size + CurrentMMArch::PAGE_SIZE - 1) / CurrentMMArch::PAGE_SIZE;
    let frames = unsafe {
        FRAME_ALLOCATOR
            .lock()
            .allocate(FrameCount::new(frame_count))
    }
    .ok_or(Errno::ENOMEM)?;

    let mut mapper = unsafe { rmm::PageMapper::current(rmm::TableKind::User, TheFrameAllocator) };
    for i in 0..frame_count {
        let flags = PageFlags::<CurrentMMArch>::new()
            .user(true)
            .write(true)
            .execute((prot & PROT_EXEC) != 0);

        if let Some(flusher) = unsafe {
            mapper.map_phys(
                virt_addr.add(i * CurrentMMArch::PAGE_SIZE),
                frames.add(i * CurrentMMArch::PAGE_SIZE),
                flags,
            )
        } {
            flusher.flush();
        }
    }

    if fd != -1_isize as usize {
        if let Some(fd_manager) = get_file_descriptor_manager() {
            if let Some((inode, _, _)) = fd_manager.file_descriptors.get(&fd) {
                let buffer =
                    unsafe { core::slice::from_raw_parts_mut(virt_addr.data() as *mut u8, size) };
                inode.read().read_at(offset as usize, buffer)?;
            }
        }
    } else {
        unsafe { core::slice::from_raw_parts_mut(virt_addr.data() as *mut u8, size) }.fill(0);
    }

    ctx_lock.mapped_regions.push((virt_addr.data(), size));

    Ok(virt_addr.data())
}
