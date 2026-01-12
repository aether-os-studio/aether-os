use core::arch::asm;

use crate::arch::cache::CacheArch;

pub struct AArch64CacheArch;

fn get_cache_line_size() -> usize {
    let mut ctr: u64;
    unsafe { asm!("mrs {0}, ctr_el0", out(reg) ctr) };
    let dminline = (ctr >> 16) & 0xf;
    let cache_line_size = 4 << dminline;
    cache_line_size
}

impl CacheArch for AArch64CacheArch {
    fn clean_range(addr: u64, size: usize) {
        let mut start = addr as usize;
        let mut end = start + size;
        let line_size = get_cache_line_size();

        start &= !(line_size - 1);
        end = (end + line_size - 1) & !(line_size - 1);

        for addr in (start..end).step_by(line_size) {
            unsafe { asm!("dc cvac, {}", in(reg) addr) };
        }
        unsafe { asm!("dsb sy") };
    }

    fn invalidate_range(addr: u64, size: usize) {
        let mut start = addr as usize;
        let mut end = start + size;
        let line_size = get_cache_line_size();

        start &= !(line_size - 1);
        end = (end + line_size - 1) & !(line_size - 1);

        for addr in (start..end).step_by(line_size) {
            unsafe { asm!("dc ivac, {}", in(reg) addr) };
        }
        unsafe { asm!("dsb sy") };
    }

    fn flush_range(addr: u64, size: usize) {
        let mut start = addr as usize;
        let mut end = start + size;
        let line_size = get_cache_line_size();

        start &= !(line_size - 1);
        end = (end + line_size - 1) & !(line_size - 1);

        for addr in (start..end).step_by(line_size) {
            unsafe { asm!("dc civac, {}", in(reg) addr) };
        }
        unsafe { asm!("dsb sy") };
    }
}
