use spin::Lazy;
use x86_64::{
    registers::control::Cr2,
    structures::idt::{InterruptDescriptorTable, InterruptStackFrame, PageFaultErrorCode},
};

use crate::{
    arch::irq::{IrqArch, IrqRegsArch},
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
    func: u64,
    errcode: u64,
    rip: u64,
    cs: u64,
    rflags: u64,
    rsp: u64,
    ss: u64,
}

#[macro_export]
macro_rules! push_context {
    () => {
        concat!(
            r#"
            sub rsp, 0x10
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
        let ret_addr = unsafe { (self.rbp as *const u64).offset(1).read_volatile() };
        ret_addr
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

extern "x86-interrupt" fn segment_not_present(frame: InterruptStackFrame, code: u64) {
    error!("Exception: Segment Not Present\n{frame:#?}");
    error!("Error Code: {code:#x}");
    panic!("Unrecoverable fault occured, halting!");
}

extern "x86-interrupt" fn general_protection_fault(frame: InterruptStackFrame, code: u64) {
    error!("Exception: General Protection Fault\n{frame:#?}");
    error!("Error Code: {code:#x}");
    x86_64::instructions::hlt();
}

extern "x86-interrupt" fn invalid_opcode(frame: InterruptStackFrame) {
    error!("Exception: Invalid Opcode\n{frame:#?}");
    x86_64::instructions::hlt();
}

extern "x86-interrupt" fn breakpoint(frame: InterruptStackFrame) {
    debug!("Exception: Breakpoint\n{frame:#?}");
}

extern "x86-interrupt" fn double_fault(frame: InterruptStackFrame, code: u64) -> ! {
    error!("Exception: Double Fault\n{frame:#?}");
    error!("Error Code: {code:#x}");
    panic!("Unrecoverable fault occured, halting!");
}

extern "x86-interrupt" fn page_fault(frame: InterruptStackFrame, code: PageFaultErrorCode) {
    warn!("Exception: Page Fault\n{frame:#?}");
    warn!("Error Code: {code:#x}");
    match Cr2::read() {
        Ok(address) => {
            warn!("Fault Address: {address:#x}");
        }
        Err(error) => {
            warn!("Invalid virtual address: {error:?}");
        }
    }
    panic!("Cannot recover from page fault, halting!");
}

pub const INTERRUPT_INDEX_OFFSET: u8 = 32;

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum InterruptIndex {
    Timer = INTERRUPT_INDEX_OFFSET,
    ApicError,
    ApicSpurious,
    Keyboard,
    Mouse,
    HpetTimer,
}

pub static IDT: Lazy<InterruptDescriptorTable> = Lazy::new(|| {
    let mut idt = InterruptDescriptorTable::new();

    idt.breakpoint.set_handler_fn(breakpoint);
    idt.segment_not_present.set_handler_fn(segment_not_present);
    idt.invalid_opcode.set_handler_fn(invalid_opcode);
    idt.page_fault.set_handler_fn(page_fault);
    idt.general_protection_fault
        .set_handler_fn(general_protection_fault);
    idt.double_fault.set_handler_fn(double_fault);

    idt[InterruptIndex::Timer as u8].set_handler_fn(timer_interrupt);

    idt
});

#[unsafe(no_mangle)]
extern "C" fn do_timer_interrupt(regs: *mut Ptrace) {
    schedule();
}

#[unsafe(naked)]
pub extern "x86-interrupt" fn timer_interrupt(_frame: InterruptStackFrame) {
    core::arch::naked_asm!(
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
