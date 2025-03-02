#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum IpiKind {
    Wakeup = 0x40,
    Tlb = 0x41,
    Switch = 0x42,
    Pit = 0x43,
}

#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum IpiTarget {
    Current = 1,
    All = 2,
    Other = 3,
}

#[inline(always)]
pub fn ipi(kind: IpiKind, target: IpiTarget) {
    use crate::arch::apic::local::the_local_apic;

    let icr = (target as u64) << 18 | 1 << 14 | (kind as u64);
    unsafe { the_local_apic().set_icr(icr) };
}

use crate::{context::scheduler::SCHEDULER, cpu_set::LogicalCpuId};

#[inline(always)]
pub fn ipi_single(kind: IpiKind, target: LogicalCpuId) {
    use crate::arch::apic::local::the_local_apic;

    unsafe {
        // TODO: Distinguish between logical and physical CPU IDs
        the_local_apic().ipi(target.get(), kind);
    }
}

use rmm::VirtualAddress;
use x86_64::structures::idt::InterruptStackFrame;
#[naked]
pub extern "x86-interrupt" fn pit(_frame: InterruptStackFrame) {
    fn pit_handler(context: VirtualAddress) -> VirtualAddress {
        super::apic::local::eoi();
        ipi(IpiKind::Pit, IpiTarget::Other);
        SCHEDULER.lock().schedule(context)
    }

    unsafe {
        core::arch::naked_asm!(
            crate::push_context!(),
            "mov rdi, rsp",
            "call {pit_handler}",
            "mov rsp, rax",
            crate::pop_context!(),
            "iretq",
            pit_handler = sym pit_handler,
        );
    }
}
