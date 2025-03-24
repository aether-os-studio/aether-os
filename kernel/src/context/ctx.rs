use x86_64::{PhysAddr, VirtAddr, structures::gdt::SegmentSelector};

#[derive(Default, Clone, Copy)]
pub struct IntrContext {
    pub rip: usize,
    pub cs: usize,
    pub rflags: usize,
    pub rsp: usize,
    pub ss: usize,
}

#[derive(Default, Clone, Copy)]
pub struct GenericContext {
    pub rbp: usize,
    pub rsi: usize,
    pub rdi: usize,

    pub rdx: usize,
    pub rcx: usize,
    pub rbx: usize,
    pub rax: usize,
}

#[derive(Default, Clone, Copy)]
pub struct ArchSpecContext {
    pub r15: usize,
    pub r14: usize,
    pub r13: usize,

    pub r12: usize,
    pub r11: usize,
    pub r10: usize,
    pub r9: usize,

    pub r8: usize,
}

#[derive(Default, Clone, Copy)]
pub struct Context {
    pub cr3: usize,

    pub arch_spec: ArchSpecContext,

    pub generic: GenericContext,

    pub intr: IntrContext,
}

impl Context {
    pub fn init(
        &mut self,
        entry: usize,
        stack_end: VirtAddr,
        page_table_addr: PhysAddr,
        segment_selector: (SegmentSelector, SegmentSelector),
        argc: usize,
        argv: usize,
        envp: usize,
    ) {
        self.intr.rflags = 0x200;
        self.intr.rip = entry;
        self.intr.rsp = stack_end.as_u64() as usize;
        self.cr3 = page_table_addr.as_u64() as usize;

        let (code_selector, data_selector) = segment_selector;
        self.intr.cs = code_selector.0 as usize;
        self.intr.ss = data_selector.0 as usize;

        self.generic.rdi = argc;
        self.generic.rsi = argv;
        self.generic.rdx = envp;
    }
}

impl Context {
    #[inline]
    pub fn address(&self) -> VirtAddr {
        VirtAddr::new(self as *const Context as u64)
    }

    #[inline]
    pub fn from_address(address: VirtAddr) -> Context {
        unsafe { *(address.as_u64() as *mut Context) }
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
