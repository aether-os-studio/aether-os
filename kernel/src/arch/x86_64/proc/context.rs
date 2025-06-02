use alloc::boxed::Box;
use goblin::elf::program_header::PT_LOAD;
use rmm::{Arch, FrameAllocator, PageFlags, PhysicalAddress, VirtualAddress};
use x86_64::{
    PhysAddr, VirtAddr,
    registers::{
        control::Cr3,
        segmentation::{FS, Segment64},
    },
    structures::paging::PhysFrame,
};

use crate::{
    arch::{CurrentMMArch, gdt::Selectors, rmm::PageMapper},
    memory::{FRAME_ALLOCATOR, KERNEL_PAGE_TABLE, frame::TheFrameAllocator},
    proc::{
        ArchContext,
        exec::{USER_STACK_END, USER_STACK_SIZE},
        sched::get_current_context,
    },
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
    regs: *mut ContextArch,
    fs: usize,
    gs: usize,
    fsbase: usize,
    gsbase: usize,
}

impl ArchContext for ContextRegs {
    fn init(&mut self, entry: usize, stack: usize) {
        let new_page_table =
            unsafe { PageMapper::create(rmm::TableKind::User, TheFrameAllocator) }.unwrap();

        for i in (CurrentMMArch::PAGE_ENTRIES / 2)..CurrentMMArch::PAGE_ENTRIES {
            unsafe {
                if let Some(entry) = KERNEL_PAGE_TABLE.lock().table().entry(i) {
                    new_page_table.table().set_entry(i, entry);
                }
            }
        }

        self.regs = unsafe { (stack as *mut ContextArch).sub(1) };
        unsafe { *self.regs = ContextArch::default() };

        self.cr3 = new_page_table.table().phys().data();
        let (code, data) = Selectors::get_kernel_segments();
        unsafe {
            (*self.regs).rip = entry;
            (*self.regs).rsp = stack;
            (*self.regs).rbp = stack;
            (*self.regs).cs = code.0 as usize;
            (*self.regs).ss = data.0 as usize;
            (*self.regs).rflags = 0x200;
        }
    }

    fn go_to_user(&mut self, entry: usize, stack: usize) {
        let (code, data) = Selectors::get_user_segments();
        unsafe {
            (*self.regs).cs = code.0 as usize;
            (*self.regs).ss = data.0 as usize;
            (*self.regs).rflags = 0x200;
            (*self.regs).rbp = stack;
            (*self.regs).rsp = stack;
            (*self.regs).rip = entry;
        }
    }

    fn clone(&self, addr: VirtualAddress, regs_addr: VirtualAddress) -> Box<dyn ArchContext> {
        let mut new_page_table =
            unsafe { PageMapper::create(rmm::TableKind::User, TheFrameAllocator) }
                .expect("Failed to create new page table");

        for i in (CurrentMMArch::PAGE_ENTRIES / 2)..CurrentMMArch::PAGE_ENTRIES {
            unsafe {
                if let Some(entry) = KERNEL_PAGE_TABLE.lock().table().entry(i) {
                    new_page_table.table().set_entry(i, entry);
                }
            }
        }

        let mut copy_mapping = |src_addr: VirtualAddress,
                                size: usize,
                                flags: PageFlags<CurrentMMArch>| {
            let count = (size + CurrentMMArch::PAGE_SIZE - 1) / CurrentMMArch::PAGE_SIZE;

            let aligned_addr =
                VirtualAddress::new(src_addr.data() & !CurrentMMArch::PAGE_OFFSET_MASK);

            for i in 0..=count {
                unsafe {
                    if let Some(flusher) =
                        new_page_table.map(aligned_addr.add(i * CurrentMMArch::PAGE_SIZE), flags)
                    {
                        flusher.ignore();
                    }
                }
            }

            let mut written: usize = 0;

            while written < size {
                let current_address = src_addr.data() + written;
                let page_offset = current_address % CurrentMMArch::PAGE_SIZE;
                let remaining = size - written;
                let chunk_size = (CurrentMMArch::PAGE_SIZE - page_offset).min(remaining) as usize;

                let physical_address = new_page_table
                    .translate(VirtualAddress::new(
                        current_address & !CurrentMMArch::PAGE_OFFSET_MASK,
                    ))
                    .unwrap()
                    .0
                    .add(page_offset);
                let virtual_address = unsafe { CurrentMMArch::phys_to_virt(physical_address) };

                unsafe {
                    core::intrinsics::copy_nonoverlapping(
                        (src_addr.data() + written) as *const u8,
                        virtual_address.data() as *mut u8,
                        chunk_size,
                    )
                };

                written += chunk_size;
            }
        };

        for phdr in get_current_context().read().memory_mappings.phdrs.iter() {
            if phdr.p_type == PT_LOAD {
                copy_mapping(
                    VirtualAddress::new(phdr.p_vaddr as usize),
                    phdr.p_memsz as usize,
                    PageFlags::new().execute(true).write(true).user(true),
                );
            }
        }

        if get_current_context()
            .read()
            .memory_mappings
            .interpreter_load_start
            != 0
        {
            copy_mapping(
                VirtualAddress::new(
                    get_current_context()
                        .read()
                        .memory_mappings
                        .interpreter_load_start,
                ),
                get_current_context()
                    .read()
                    .memory_mappings
                    .interpreter_load_end
                    - get_current_context()
                        .read()
                        .memory_mappings
                        .interpreter_load_start,
                PageFlags::new().execute(true).write(true).user(true),
            );
        }

        copy_mapping(
            VirtualAddress::new(get_current_context().read().memory_mappings.brk_start),
            get_current_context().read().memory_mappings.brk_end
                - get_current_context().read().memory_mappings.brk_start,
            PageFlags::new().execute(true).write(true).user(true),
        );

        copy_mapping(
            VirtualAddress::new(USER_STACK_END - USER_STACK_SIZE),
            USER_STACK_SIZE,
            PageFlags::new().write(true).user(true),
        );

        for region in get_current_context()
            .read()
            .memory_mappings
            .mapped_regions
            .iter()
        {
            copy_mapping(
                VirtualAddress::new(region.0),
                region.1,
                PageFlags::new().write(true).user(true),
            );
        }

        let regs = unsafe {
            let context = (addr.data() as *mut ContextArch).sub(1);

            core::intrinsics::copy_nonoverlapping(
                regs_addr.data() as *const ContextArch,
                context,
                1,
            );

            context
        };

        let new_context = ContextRegs {
            cr3: new_page_table.table().phys().data(),
            regs,
            fs: self.fs,
            gs: self.gs,
            fsbase: self.fsbase,
            gsbase: self.gsbase,
        };

        unsafe { (*new_context.regs).rax = 0 };

        Box::new(new_context)
    }

    fn address(&self) -> VirtualAddress {
        VirtualAddress::new(self.regs as usize)
    }

    fn set_address(&mut self, addr: VirtualAddress) {
        self.regs = addr.data() as *mut ContextArch;
    }

    fn page_table_address(&self) -> PhysicalAddress {
        PhysicalAddress::new(self.cr3)
    }

    fn make_current(&self) {
        unsafe { FS::write_base(VirtAddr::new(self.fsbase as u64)) };

        // page table
        unsafe {
            Cr3::write(
                PhysFrame::containing_address(PhysAddr::new(self.cr3 as u64)),
                Cr3::read().1,
            )
        };
    }

    fn exit(&self) {
        let mut page_table = unsafe {
            PageMapper::new(
                rmm::TableKind::User,
                PhysicalAddress::new(self.cr3),
                TheFrameAllocator,
            )
        };

        for i in 0..(CurrentMMArch::PAGE_ENTRIES / 2) {
            unsafe {
                if let Some(entry) = page_table.table().entry(i) {
                    if entry.present() {
                        page_table.unmap(VirtualAddress::new(i * CurrentMMArch::PAGE_SIZE), true);
                    }
                }
            }
        }

        unsafe { FRAME_ALLOCATOR.lock().free_one(page_table.table().phys()) };
    }

    fn get_fs(&self) -> usize {
        self.fsbase
    }

    fn set_fs(&mut self, fsbase: usize) {
        self.fsbase = fsbase;
    }
}

pub unsafe fn arch_to_user_mode(regs_addr: VirtualAddress) {
    core::arch::asm!("mov rsp, {}", crate::pop_context!(), "iretq", in(reg) regs_addr.data(), options(nostack, noreturn));
}
