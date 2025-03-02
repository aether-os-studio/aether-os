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

use crate::cpu_set::LogicalCpuId;

#[inline(always)]
pub fn ipi_single(kind: IpiKind, target: LogicalCpuId) {
    use crate::arch::apic::local::the_local_apic;

    unsafe {
        // TODO: Distinguish between logical and physical CPU IDs
        the_local_apic().ipi(target.get(), kind);
    }
}

use x86_64::structures::idt::InterruptStackFrame;

pub extern "x86-interrupt" fn pit(_frame: InterruptStackFrame) {
    crate::arch::apic::local::eoi();
    log::debug!("pit ipi");
}
