use core::{
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
};

use rmm::{Arch, FrameAllocator, FrameCount, PhysicalAddress};

use crate::{
    arch::CurrentRmmArch,
    init::memory::{FRAME_ALLOCATOR, PAGE_SIZE},
};

pub struct Dma<T: ?Sized> {
    phys: PhysicalAddress,
    count: FrameCount,
    virt: *mut T,
}

impl<T> Dma<T> {
    pub fn new(value: T) -> Option<Self> {
        unsafe {
            let mut zeroed = Self::zeroed()?;
            zeroed.as_mut_ptr().write(value);
            Some(zeroed.assume_init())
        }
    }

    pub fn zeroed() -> Option<Dma<MaybeUninit<T>>> {
        let aligned_len = size_of::<T>().next_multiple_of(PAGE_SIZE);
        let count = FrameCount::new(aligned_len / PAGE_SIZE);
        let phys = unsafe { FRAME_ALLOCATOR.lock().allocate(count) }?;
        let virt = unsafe { CurrentRmmArch::phys_to_virt(phys) };

        unsafe { core::ptr::write_bytes(virt.data() as *mut u8, 0, aligned_len) };

        Some(Dma {
            phys,
            virt: virt.data() as *mut MaybeUninit<T>,
            count,
        })
    }
}

impl<T> Dma<MaybeUninit<T>> {
    pub unsafe fn assume_init(self) -> Dma<T> {
        let Dma { phys, count, virt } = self;

        core::mem::forget(self);

        Dma {
            phys,
            count,
            virt: virt.cast(),
        }
    }
}

impl<T: ?Sized> Dma<T> {
    pub fn physical(&self) -> PhysicalAddress {
        self.phys
    }
}
impl<T> Dma<[T]> {
    pub fn zeroed_slice(count: usize) -> Option<Dma<[MaybeUninit<T>]>> {
        let aligned_len = count
            .checked_mul(size_of::<T>())
            .unwrap()
            .next_multiple_of(PAGE_SIZE);
        let count = FrameCount::new(aligned_len / PAGE_SIZE);
        let phys = unsafe { FRAME_ALLOCATOR.lock().allocate(count) }?;
        let virt = unsafe { CurrentRmmArch::phys_to_virt(phys) };

        unsafe { core::ptr::write_bytes(virt.data() as *mut u8, 0, aligned_len) };

        Some(Dma {
            phys,
            count,
            virt: core::ptr::slice_from_raw_parts_mut(
                virt.data() as *mut MaybeUninit<T>,
                aligned_len,
            ),
        })
    }

    pub unsafe fn cast_slice<U>(self) -> Dma<[U]> {
        let Dma { phys, virt, count } = self;

        core::mem::forget(self);

        Dma {
            phys,
            virt: virt as *mut [U],
            count,
        }
    }
}

impl<T> Dma<[MaybeUninit<T>]> {
    pub unsafe fn assume_init(self) -> Dma<[T]> {
        let &Dma { phys, count, virt } = &self;

        core::mem::forget(self);

        Dma {
            phys,
            count,
            virt: virt as *mut [T],
        }
    }
}

impl<T: ?Sized> Deref for Dma<T> {
    type Target = T;

    fn deref(&self) -> &T {
        unsafe { &*self.virt }
    }
}

impl<T: ?Sized> DerefMut for Dma<T> {
    fn deref_mut(&mut self) -> &mut T {
        unsafe { &mut *self.virt }
    }
}

impl<T: ?Sized> Drop for Dma<T> {
    fn drop(&mut self) {
        unsafe {
            core::ptr::drop_in_place(self.virt);
            FRAME_ALLOCATOR.lock().free(self.physical(), self.count);
        }
    }
}

unsafe impl<T> Send for Dma<T> {}
unsafe impl<T> Sync for Dma<T> {}
