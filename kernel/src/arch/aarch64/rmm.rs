use core::arch::asm;

use rmm::{Arch, PageFlags, VirtualAddress};

const MAIR_ATTR_DEVICE_NGNRNE: u64 = 0x00;
const MAIR_ATTR_DEVICE_NGNRE: u64 = 0x04;
const MAIR_ATTR_NORMAL_NC: u64 = 0x44;
const MAIR_ATTR_NORMAL_WT: u64 = 0xBB;
const MAIR_ATTR_NORMAL_WB: u64 = 0xFF;

pub fn init() {
    let mair_val =
        (MAIR_ATTR_NORMAL_WB << 0) | (MAIR_ATTR_NORMAL_NC << 8) | (MAIR_ATTR_DEVICE_NGNRNE << 16);
    unsafe {
        asm!("msr mair_el1, {}", in(reg) mair_val);
    }
}

pub unsafe fn page_flags<A: Arch>(virt: VirtualAddress) -> PageFlags<A> {
    use crate::kernel_executable_offsets::*;
    let virt_addr = virt.data();

    if virt_addr >= __text_start() && virt_addr < __text_end() {
        PageFlags::new().execute(true)
    } else if virt_addr >= __rodata_start() && virt_addr < __rodata_end() {
        PageFlags::new()
    } else {
        PageFlags::new().write(true)
    }
}
