use rmm::{Arch, PageFlags, VirtualAddress};

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
