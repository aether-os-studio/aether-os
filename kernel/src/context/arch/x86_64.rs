use rmm::{PageMapper, PhysicalAddress, TableKind, VirtualAddress};
use syscall::{Error, Result, ENOMEM};
use x86_64::structures::gdt::SegmentSelector;

use super::memory::Table;

#[derive(Debug, Clone, Copy, Default)]
#[repr(C, packed)]
#[allow(dead_code)]
pub struct Context {
    cr3: usize,
    r15: usize,
    r14: usize,
    r13: usize,

    r12: usize,
    r11: usize,
    r10: usize,
    r9: usize,

    r8: usize,
    rbp: usize,
    rsi: usize,
    rdi: usize,

    rdx: usize,
    rcx: usize,
    rbx: usize,
    rax: usize,

    rip: usize,
    cs: usize,
    rflags: usize,
    rsp: usize,
    ss: usize,
}

impl Context {
    pub fn init(
        &mut self,
        entry_point: usize,
        stack_end_address: VirtualAddress,
        page_table_address: PhysicalAddress,
        segment_selectors: (SegmentSelector, SegmentSelector),
    ) {
        self.rflags = 0x200;
        self.rip = entry_point;
        self.rsp = stack_end_address.data();
        self.cr3 = page_table_address.data();

        let (code_selector, data_selector) = segment_selectors;
        self.cs = code_selector.0 as usize;
        self.ss = data_selector.0 as usize;
    }
}

impl Context {
    #[inline]
    pub fn address(&self) -> VirtualAddress {
        VirtualAddress::new(self as *const Context as usize)
    }

    #[inline]
    pub fn from_address(address: VirtualAddress) -> Context {
        unsafe { *(address.data() as *mut Context) }
    }
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
            mov r15, cr3
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
            mov cr3, r15
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

/// Allocates a new identically mapped ktable and empty utable (same memory on x86_64).
pub fn setup_new_utable() -> Result<Table> {
    use crate::memory::{mapper::KernelMapper, TheFrameAllocator};

    let utable = unsafe {
        PageMapper::create(TableKind::User, TheFrameAllocator).ok_or(Error::new(ENOMEM))?
    };

    {
        let active_ktable = KernelMapper::lock();

        let copy_mapping = |p4_no| unsafe {
            let entry = active_ktable
                .table()
                .entry(p4_no)
                .unwrap_or_else(|| panic!("expected kernel PML {} to be mapped", p4_no));

            utable.table().set_entry(p4_no, entry)
        };
        // TODO: Just copy all 256 mappings? Or copy KERNEL_PML4+KERNEL_PERCPU_PML4 (needed for
        // paranoid ISRs which can occur anywhere; we don't want interrupts to triple fault!) and
        // map lazily via page faults in the kernel.

        // Copy kernel image mapping
        log::debug!("Coping kernel file space");
        copy_mapping(crate::memory::KERNEL_FILE_PML4);

        // Copy kernel heap mapping
        log::debug!("Coping kernel heap space");
        copy_mapping(crate::memory::KERNEL_HEAP_PML4);

        // Copy physmap mapping
        log::debug!("Coping kernel memory space");
        copy_mapping(crate::memory::PHYS_PML4);
    }

    Ok(Table { utable })
}
