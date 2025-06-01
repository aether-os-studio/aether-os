use rmm::VirtualAddress;
use spin::Lazy;
use x86_64::{
    registers::control::Cr2,
    set_general_handler,
    structures::idt::{InterruptDescriptorTable, InterruptStackFrame, PageFaultErrorCode},
};

use crate::{proc::sched::SCHEDULER, serial_println};

use super::gdt::DOUBLE_FAULT_IST_INDEX;

const INTERRUPT_INDEX_OFFSET: u8 = 32;

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum InterruptIndex {
    Timer = INTERRUPT_INDEX_OFFSET,
    ApicError,
    ApicSpurious,
}

pub static IDT: Lazy<InterruptDescriptorTable> = Lazy::new(|| {
    let mut idt = InterruptDescriptorTable::new();

    set_general_handler!(&mut idt, x64_do_irq);

    idt.breakpoint.set_handler_fn(breakpoint);
    idt.segment_not_present.set_handler_fn(segment_not_present);
    idt.invalid_opcode.set_handler_fn(invalid_opcode);
    idt.page_fault.set_handler_fn(page_fault);
    idt.general_protection_fault
        .set_handler_fn(general_protection_fault);

    idt[InterruptIndex::Timer as u8].set_handler_fn(timer_interrupt);
    idt[InterruptIndex::ApicError as u8].set_handler_fn(lapic_error);
    idt[InterruptIndex::ApicSpurious as u8].set_handler_fn(spurious_interrupt);

    unsafe {
        idt.double_fault
            .set_handler_fn(double_fault)
            .set_stack_index(DOUBLE_FAULT_IST_INDEX as u16);
    }

    return idt;
});

fn x64_do_irq(frame: InterruptStackFrame, index: u8, error_code: Option<u64>) {
    serial_println!("Unhandled IRQ {}", index);
}

#[unsafe(naked)]
extern "x86-interrupt" fn timer_interrupt(_frame: InterruptStackFrame) {
    fn timer_handler(context: VirtualAddress) -> VirtualAddress {
        super::apic::end_of_interrupt();
        SCHEDULER.lock().schedule(context)
    }

    core::arch::naked_asm!(
        crate::push_context!(),
        "mov rdi, rsp",
        "call {timer_handler}",
        "mov rsp, rax",
        crate::pop_context!(),
        "iretq",
        timer_handler = sym timer_handler,
    );
}

extern "x86-interrupt" fn lapic_error(_frame: InterruptStackFrame) {
    serial_println!("Local APIC error!");
    super::apic::end_of_interrupt();
}

extern "x86-interrupt" fn spurious_interrupt(_frame: InterruptStackFrame) {
    serial_println!("Received spurious interrupt!");
    super::apic::end_of_interrupt();
}

extern "x86-interrupt" fn segment_not_present(frame: InterruptStackFrame, error_code: u64) {
    serial_println!("Exception: Segment Not Present\n{:#?}", frame);
    serial_println!("Error Code: {:#x}", error_code);
    panic!("Unrecoverable fault occured, halting!");
}

extern "x86-interrupt" fn general_protection_fault(frame: InterruptStackFrame, error_code: u64) {
    serial_println!("Exception: General Protection Fault\n{:#?}", frame);
    serial_println!("Error Code: {:#x}", error_code);
    x86_64::instructions::hlt();
}

extern "x86-interrupt" fn invalid_opcode(frame: InterruptStackFrame) {
    serial_println!("Exception: Invalid Opcode\n{:#?}", frame);
    x86_64::instructions::hlt();
}

extern "x86-interrupt" fn breakpoint(frame: InterruptStackFrame) {
    serial_println!("Exception: Breakpoint\n{:#?}", frame);
}

extern "x86-interrupt" fn double_fault(frame: InterruptStackFrame, error_code: u64) -> ! {
    serial_println!("Exception: Double Fault\n{:#?}", frame);
    serial_println!("Error Code: {:#x}", error_code);
    panic!("Unrecoverable fault occured, halting!");
}

extern "x86-interrupt" fn page_fault(frame: InterruptStackFrame, error_code: PageFaultErrorCode) {
    serial_println!("Exception: Page Fault\n{:#?}", frame);
    serial_println!("Error Code: {:#x}", error_code);
    match Cr2::read() {
        Ok(address) => {
            serial_println!("Fault Address: {:#x}", address);
        }
        Err(error) => {
            serial_println!("Invalid virtual address: {:?}", error);
        }
    }
    serial_println!("{:#?}", error_code);

    x86_64::instructions::hlt();
}

pub fn init() {
    IDT.load();
}
