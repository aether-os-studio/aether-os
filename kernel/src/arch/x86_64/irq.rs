use spin::Lazy;
use x86_64::{
    registers::control::Cr2,
    structures::idt::{InterruptDescriptorTable, PageFaultErrorCode},
};

use crate::{
    arch::{
        drivers::apic::LAPIC,
        gdt::Selectors,
        irq::{IrqArch, IrqRegsArch},
    },
    task::schedule,
};

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Ptrace {
    r15: u64,
    r14: u64,
    r13: u64,
    r12: u64,
    r11: u64,
    r10: u64,
    r9: u64,
    r8: u64,
    rbx: u64,
    rcx: u64,
    rdx: u64,
    rsi: u64,
    rdi: u64,
    rbp: u64,
    rax: u64,
    reserved: u64,
    errcode: u64,
    rip: u64,
    cs: u64,
    rflags: u64,
    rsp: u64,
    ss: u64,
}
impl core::fmt::Display for Ptrace {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "r15: {:#x}", self.r15)?;
        writeln!(f, "r14: {:#x}", self.r14)?;
        writeln!(f, "r13: {:#x}", self.r13)?;
        writeln!(f, "r12: {:#x}", self.r12)?;
        writeln!(f, "r11: {:#x}", self.r11)?;
        writeln!(f, "r10: {:#x}", self.r10)?;
        writeln!(f, "r9: {:#x}", self.r9)?;
        writeln!(f, "r8: {:#x}", self.r8)?;
        writeln!(f, "rbx: {:#x}", self.rbx)?;
        writeln!(f, "rcx: {:#x}", self.rcx)?;
        writeln!(f, "rdx: {:#x}", self.rdx)?;
        writeln!(f, "rsi: {:#x}", self.rsi)?;
        writeln!(f, "rdi: {:#x}", self.rdi)?;
        writeln!(f, "rbp: {:#x}", self.rbp)?;
        writeln!(f, "rax: {:#x}", self.rax)?;
        writeln!(f, "rip: {:#x}", self.rip)?;
        writeln!(f, "cs: {:#x}", self.cs)?;
        writeln!(f, "rflags: {:#x}", self.rflags)?;
        writeln!(f, "rsp: {:#x}", self.rsp)?;
        write!(f, "ss: {:#x}", self.ss)?;
        Ok(())
    }
}

#[macro_export]
macro_rules! push_context {
    () => {
        concat!(
            r#"
            sub rsp, 0x8
            push rax
            push rbp
            push rdi
            push rsi
            push rdx
            push rcx
            push rbx
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
            pop rbx
            pop rcx
            pop rdx
            pop rsi
            pop rdi
            pop rbp
            pop rax
            add rsp, 0x10
			"#
        )
    };
}

impl IrqRegsArch for Ptrace {
    fn get_ip(&self) -> u64 {
        self.rip
    }

    fn set_ip(&mut self, ip: u64) {
        self.rip = ip;
    }

    fn get_sp(&self) -> u64 {
        self.rsp
    }

    fn set_sp(&mut self, sp: u64) {
        self.rsp = sp;
    }

    fn get_ret_value(&self) -> u64 {
        self.rax
    }

    fn set_ret_value(&mut self, ret_value: u64) {
        self.rax = ret_value
    }

    fn get_ret_address(&self) -> u64 {
        unsafe { (self.rbp as *const u64).offset(1).read_volatile() }
    }

    fn set_ret_address(&mut self, ret_address: u64) {
        unsafe { (self.rbp as *mut u64).offset(1).write_volatile(ret_address) };
    }

    fn get_syscall_idx(&self) -> u64 {
        self.rax
    }

    fn get_syscall_args(&self) -> (u64, u64, u64, u64, u64, u64) {
        (self.rdi, self.rsi, self.rdx, self.r10, self.r8, self.r9)
    }

    fn get_args(&self) -> (u64, u64, u64, u64, u64, u64) {
        (self.rdi, self.rsi, self.rdx, self.rcx, self.r8, self.r9)
    }

    fn set_args(&mut self, args: (u64, u64, u64, u64, u64, u64)) {
        self.rdi = args.0;
        self.rsi = args.1;
        self.rdx = args.2;
        self.rcx = args.3;
        self.r8 = args.4;
        self.r9 = args.5;
    }

    fn set_user_space(&mut self, user: bool) {
        self.rflags = 0x202;
        let (code, data) = if user {
            Selectors::get_user_segments()
        } else {
            Selectors::get_kernel_segments()
        };
        self.cs = code.0 as u64;
        self.ss = data.0 as u64;
    }
}

pub struct X8664IrqArch;

impl IrqArch for X8664IrqArch {
    fn enable_global_irq() {
        x86_64::instructions::interrupts::enable();
    }

    fn disable_global_irq() {
        x86_64::instructions::interrupts::disable();
    }
}

#[unsafe(no_mangle)]
extern "C" fn do_general_protection_fault(regs: *mut Ptrace) {
    let regs = unsafe { regs.as_mut_unchecked() };
    error!("Exception: General Protection Fault");
    panic!("{}", regs);
}

#[unsafe(naked)]
extern "C" fn general_protection_fault() {
    core::arch::naked_asm!(
        push_context!(),
        "mov rdi, rsp",
        "call do_general_protection_fault",
        pop_context!(),
        "iretq",
    );
}

#[unsafe(no_mangle)]
extern "C" fn do_invalid_opcode(regs: *mut Ptrace) {
    let regs = unsafe { regs.as_mut_unchecked() };
    error!("Exception: Invalid Opcode");
    panic!("{}", regs);
}

#[unsafe(naked)]
extern "C" fn invalid_opcode() -> ! {
    core::arch::naked_asm!(
        "sub rsp, 0x8",
        push_context!(),
        "mov rdi, rsp",
        "call do_invalid_opcode",
        pop_context!(),
        "iretq",
    );
}

#[unsafe(no_mangle)]
extern "C" fn do_double_fault(regs: *mut Ptrace) {
    let regs = unsafe { regs.as_mut_unchecked() };
    error!("Exception: Double Fault");
    panic!("{}\nUnrecoverable fault occured, halting!", regs);
}

#[unsafe(naked)]
extern "C" fn double_fault() -> ! {
    core::arch::naked_asm!(
        push_context!(),
        "mov rdi, rsp",
        "call do_double_fault",
        pop_context!(),
        "iretq",
    );
}

#[unsafe(no_mangle)]
extern "C" fn do_page_fault(regs: *mut Ptrace) {
    let regs = unsafe { regs.as_mut_unchecked() };
    warn!("Exception: Page Fault");
    let page_fault_errcode = PageFaultErrorCode::from_bits_truncate(regs.errcode);
    warn!("Page Fault Error Code: {:#?}", page_fault_errcode);
    match Cr2::read() {
        Ok(address) => {
            warn!("Fault Address: {address:#x}");
        }
        Err(error) => {
            warn!("Invalid virtual address: {error:?}");
        }
    }
    panic!("{}", regs);
}

#[unsafe(naked)]
extern "C" fn page_fault() {
    core::arch::naked_asm!(
        push_context!(),
        "mov rdi, rsp",
        "call do_page_fault",
        pop_context!(),
        "iretq",
    );
}

pub const INTERRUPT_INDEX_OFFSET: u8 = 32;

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum InterruptIndex {
    Timer = INTERRUPT_INDEX_OFFSET,
    ApicError,
    ApicSpurious,
}

pub static IDT: Lazy<InterruptDescriptorTable> = Lazy::new(|| {
    let mut idt = InterruptDescriptorTable::new();

    unsafe {
        idt.invalid_opcode
            .set_handler_addr(x86_64::VirtAddr::new(invalid_opcode as *const () as u64));
        idt.page_fault
            .set_handler_addr(x86_64::VirtAddr::new(page_fault as *const () as u64));
        idt.general_protection_fault
            .set_handler_addr(x86_64::VirtAddr::new(
                general_protection_fault as *const () as u64,
            ));
        idt.double_fault
            .set_handler_addr(x86_64::VirtAddr::new(double_fault as *const () as u64));

        idt[InterruptIndex::Timer as u8]
            .set_handler_addr(x86_64::VirtAddr::new(timer_interrupt as *const () as u64));
    }

    idt
});

#[unsafe(no_mangle)]
extern "C" fn do_timer_interrupt(_regs: *mut Ptrace) {
    if let Some(lapic) = LAPIC.lock().as_mut() {
        unsafe { lapic.end_of_interrupt() };
    }
    schedule();
}

#[unsafe(naked)]
pub extern "C" fn kernel_thread_entry() {
    core::arch::naked_asm!(
        pop_context!(),
        "add rsp, 0x28",
        "call rdx",
        "mov rdi, rax",
        "jmp {exit_current}",
        exit_current = sym crate::task::exit_current,
    );
}

#[unsafe(naked)]
pub extern "C" fn return_from_interrupt() {
    core::arch::naked_asm!(pop_context!(), "iretq");
}

#[unsafe(naked)]
pub extern "C" fn timer_interrupt() {
    core::arch::naked_asm!(
        "sub rsp, 0x8",
        push_context!(),
        "mov rdi, rsp",
        "call do_timer_interrupt",
        pop_context!(),
        "iretq",
    );
}

pub fn init() {
    IDT.load();
}
