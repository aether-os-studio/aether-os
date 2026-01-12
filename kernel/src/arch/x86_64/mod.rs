mod boot;
pub mod cache;
pub mod drivers;
pub mod gdt;
pub mod irq;
pub mod rmm;
pub mod smp;
pub mod time;

use crate::arch::smp::LAPICID_TO_CPUINFO;
use crate::task::ArcTask;
use crate::task::Task;

pub use self::cache::X8664CacheArch as CurrentCacheArch;
pub use self::irq::Ptrace;
pub use self::irq::X8664IrqArch as CurrentIrqArch;
pub use self::smp::get_lapicid as get_archid;
pub use self::time::X8664TimeArch as CurrentTimeArch;
use ::rmm::Arch;
use ::rmm::TableKind;
pub use ::rmm::X8664Arch as CurrentRmmArch;
use x86_64::registers::control::{Cr0, Cr0Flags, Cr4, Cr4Flags};

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ArchContext {
    pub ip: usize,
    pub sp: usize,
    pub fsbase: usize,
    pub gsbase: usize,
}

#[unsafe(no_mangle)]
unsafe extern "C" fn do_switch_to(prev: *mut Task, next: *const Task) {
    let prev = prev.as_mut_unchecked();
    let next = next.as_ref_unchecked();

    if prev.page_table_addr != next.page_table_addr {
        CurrentRmmArch::set_table(TableKind::User, next.page_table_addr);
    }
}

use core::mem::offset_of;

#[unsafe(naked)]
pub extern "C" fn switch_to_inner(prev: *mut Task, next: *const Task) {
    core::arch::naked_asm!(
        "push rbp",
        "mov [rdi + {sp_off}], rsp",
        "mov rsp, [rsi + {sp_off}]",
        "lea rax, [rip + 1f]",
        "mov [rdi + {ip_off}], rax",
        "push qword ptr [rsi + {ip_off}]",
        "jmp do_switch_to",
        "1:",
        "pop rbp",
        "ret",
        sp_off = const(offset_of!(Task, arch_context) + offset_of!(ArchContext, sp)),
        ip_off = const(offset_of!(Task, arch_context) + offset_of!(ArchContext, ip)),
    )
}

pub fn switch_to(prev: ArcTask, next: ArcTask) {
    LAPICID_TO_CPUINFO
        .lock()
        .get_mut(&get_archid())
        .unwrap()
        .set_ring0_rsp(next.read().get_kernel_stack_top().data() as u64);
    let prev = prev.as_mut_ptr();
    let next = next.as_mut_ptr() as *const _;
    switch_to_inner(prev, next);
}

pub fn init_sse() {
    let mut cr0 = Cr0::read();
    cr0.remove(Cr0Flags::EMULATE_COPROCESSOR);
    cr0.insert(Cr0Flags::MONITOR_COPROCESSOR);
    unsafe { Cr0::write(cr0) };

    let mut cr4 = Cr4::read();
    cr4.insert(Cr4Flags::OSFXSR);
    cr4.insert(Cr4Flags::OSXMMEXCPT_ENABLE);
    unsafe { Cr4::write(cr4) };
}

pub fn early_init() {
    init_sse();
    crate::smp::init();
    crate::arch::x86_64::irq::init();
    crate::arch::x86_64::drivers::apic::init();
}
