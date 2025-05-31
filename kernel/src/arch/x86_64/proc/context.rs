use alloc::boxed::Box;
use rmm::{PhysicalAddress, VirtualAddress};
use x86_64::{
    VirtAddr,
    registers::segmentation::{FS, Segment64},
};

use crate::{
    arch::{gdt::Selectors, rmm::PageMapper},
    memory::{KERNEL_PAGE_TABLE, frame::TheFrameAllocator},
    proc::ArchContext,
};

#[repr(C, packed)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ContextArch {
    pub r15: usize,
    pub r14: usize,
    pub r13: usize,

    pub r12: usize,
    pub r11: usize,
    pub r10: usize,
    pub r9: usize,

    pub r8: usize,
    pub rbp: usize,
    pub rsi: usize,
    pub rdi: usize,

    pub rdx: usize,
    pub rcx: usize,
    pub rbx: usize,
    pub rax: usize,

    pub rip: usize,
    pub cs: usize,
    pub rflags: usize,
    pub rsp: usize,
    pub ss: usize,
}

#[macro_export]
macro_rules! push_context {
    () => {
        concat!(
            r#"
			push rax
            push rbx
            push rcx
            push rdx
            push rdi
            push rsi
            push rbp
            push r8
            push r9
            push r10
            push r11
            push r12
            push r13
            push r14
            push r15
			"#,
        )
    };
}

#[macro_export]
macro_rules! pop_context {
    () => {
        concat!(
            r#"
			pop r15
            pop r14
            pop r13
            pop r12
            pop r11
            pop r10
            pop r9
            pop r8
            pop rbp
            pop rsi
            pop rdi
            pop rdx
            pop rcx
            pop rbx
            pop rax
			"#
        )
    };
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ContextRegs {
    cr3: usize,
    regs: ContextArch,
    fs: usize,
    gs: usize,
    fsbase: usize,
    gsbase: usize,
}

impl ArchContext for ContextRegs {
    fn init(&mut self, entry: usize, stack: usize) {
        let new_page_table =
            unsafe { PageMapper::create(rmm::TableKind::User, TheFrameAllocator) }.unwrap();

        for i in 256..512 {
            unsafe {
                if let Some(entry) = KERNEL_PAGE_TABLE.lock().table().entry(i) {
                    new_page_table.table().set_entry(i, entry);
                }
            }
        }

        self.cr3 = new_page_table.table().phys().data();
        self.regs.rip = entry;
        self.regs.rsp = stack;
        self.regs.rbp = stack;
        let (code, data) = Selectors::get_kernel_segments();
        self.regs.cs = code.0 as usize;
        self.regs.ss = data.0 as usize;
        self.regs.rflags = 0x200;
    }

    fn go_to_user(&mut self) {
        let (code, data) = Selectors::get_user_segments();
        self.regs.cs = code.0 as usize;
        self.regs.ss = data.0 as usize;
        self.regs.rflags = 0x200246;
    }

    fn clone(&self) -> Box<dyn ArchContext> {
        let new_regs = self.regs.clone();

        let new_page_table = unsafe { PageMapper::create(rmm::TableKind::User, TheFrameAllocator) }
            .expect("Failed to create new page table");

        Box::new(ContextRegs {
            cr3: new_page_table.table().phys().data(),
            regs: new_regs,
            fs: self.fs,
            gs: self.gs,
            fsbase: self.fsbase,
            gsbase: self.gsbase,
        })
    }

    fn address(&self) -> VirtualAddress {
        VirtualAddress::new(&self.regs as *const ContextArch as usize)
    }

    fn set_address(&mut self, addr: VirtualAddress) {
        let context = unsafe { *(addr.data() as *const ContextArch) };
        self.regs = context;
    }

    fn page_table_address(&self) -> PhysicalAddress {
        PhysicalAddress::new(self.cr3)
    }

    fn make_current(&self) {
        unsafe { FS::write_base(VirtAddr::new(self.fsbase as u64)) };
    }
}
