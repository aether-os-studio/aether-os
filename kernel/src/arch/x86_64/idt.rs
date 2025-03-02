use rmm::VirtualAddress;
use spin::Lazy;
use x86_64::{
    registers::control::Cr2,
    set_general_handler,
    structures::idt::{InterruptDescriptorTable, InterruptStackFrame, PageFaultErrorCode},
};

use crate::{
    arch::ipi::{ipi, IpiKind, IpiTarget},
    context::scheduler::SCHEDULER,
};

use super::gdt::DOUBLE_FAULT_IST_INDEX;

pub static IDT: Lazy<InterruptDescriptorTable> = Lazy::new(|| {
    let mut idt = InterruptDescriptorTable::new();

    set_general_handler!(&mut idt, no_irq, 0..=255);

    idt.breakpoint.set_handler_fn(breakpoint);
    idt.segment_not_present.set_handler_fn(segment_not_present);
    idt.invalid_opcode.set_handler_fn(invalid_opcode);
    idt.page_fault.set_handler_fn(page_fault);
    idt.general_protection_fault
        .set_handler_fn(general_protection_fault);

    unsafe {
        idt.double_fault
            .set_handler_fn(double_fault)
            .set_stack_index(DOUBLE_FAULT_IST_INDEX as u16);
    }

    idt[32].set_handler_fn(timer_interrupt);

    idt[IpiKind::Pit as u8].set_handler_fn(crate::arch::ipi::pit);

    idt
});

pub fn init() {
    IDT.load();
}

fn no_irq(_stack_frame: InterruptStackFrame, index: u8, _error_code: Option<u64>) {
    crate::arch::apic::local::eoi();
    log::debug!("Unhandled irq {}", index)
}

#[naked]
pub extern "x86-interrupt" fn timer_interrupt(_frame: InterruptStackFrame) {
    fn timer_handler(context: VirtualAddress) -> VirtualAddress {
        super::apic::local::eoi();
        ipi(IpiKind::Pit, IpiTarget::Other);
        SCHEDULER.lock().schedule(context)
    }

    unsafe {
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
}

extern "x86-interrupt" fn segment_not_present(frame: InterruptStackFrame, error_code: u64) {
    log::error!("Exception: Segment Not Present\n{:#?}", frame);
    log::error!("Error Code: {:#x}", error_code);
    panic!("Unrecoverable fault occured, halting!");
}

extern "x86-interrupt" fn general_protection_fault(frame: InterruptStackFrame, error_code: u64) {
    log::error!("Exception: General Protection Fault\n{:#?}", frame);
    log::error!("Error Code: {:#x}", error_code);
    x86_64::instructions::hlt();
}

extern "x86-interrupt" fn invalid_opcode(frame: InterruptStackFrame) {
    log::error!("Exception: Invalid Opcode\n{:#?}", frame);
    x86_64::instructions::hlt();
}

extern "x86-interrupt" fn breakpoint(frame: InterruptStackFrame) {
    log::debug!("Exception: Breakpoint\n{:#?}", frame);
}

extern "x86-interrupt" fn double_fault(frame: InterruptStackFrame, error_code: u64) -> ! {
    log::error!("Exception: Double Fault\n{:#?}", frame);
    log::error!("Error Code: {:#x}", error_code);
    panic!("Unrecoverable fault occured, halting!");
}

extern "x86-interrupt" fn page_fault(frame: InterruptStackFrame, error_code: PageFaultErrorCode) {
    log::warn!("Exception: Page Fault\n{:#?}", frame);
    log::warn!("Error Code: {:#x}", error_code);
    match Cr2::read() {
        Ok(address) => {
            log::warn!("Fault Address: {:#x}", address);
        }
        Err(error) => {
            log::warn!("Invalid virtual address: {:?}", error);
        }
    }
}
