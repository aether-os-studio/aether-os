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
use x86_64::registers::model_specific::FsBase;
use x86_64::registers::model_specific::GsBase;

#[repr(C, align(16))]
#[derive(Debug, Copy, Clone, Default)]
pub struct FpState {
    // 0
    fcw: u16,
    fsw: u16,
    ftw: u16,
    fop: u16,
    word2: u64,
    // 16
    word3: u64,
    mxcsr: u32,
    mxcsr_mask: u32,
    // 32
    mm: [u64; 16],
    // 160
    xmm: [u64; 32],
    // 416
    rest: [u64; 12],
}

impl FpState {
    pub fn new() -> Self {
        assert!(core::mem::size_of::<Self>() == 512);
        Self {
            mxcsr: 0x1f80,
            fcw: 0x037f,
            ..Self::default()
        }
    }

    pub fn save(&mut self) {
        unsafe {
            core::arch::x86_64::_fxsave64(self as *mut FpState as *mut u8);
        }
    }

    pub fn restore(&self) {
        unsafe {
            core::arch::x86_64::_fxrstor64(self as *const FpState as *const u8);
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ArchContext {
    pub ip: usize,
    pub sp: usize,
    pub fsbase: usize,
    pub gsbase: usize,
    pub fpu: FpState,
}

#[unsafe(no_mangle)]
unsafe extern "C" fn do_switch_to(prev: *mut Task, next: *const Task) {
    let prev = prev.as_mut_unchecked();
    let next = next.as_ref_unchecked();

    if prev.page_table_addr != next.page_table_addr {
        CurrentRmmArch::set_table(TableKind::User, next.page_table_addr);
    }

    prev.arch_context.fsbase = FsBase::read().as_u64() as usize;
    prev.arch_context.gsbase = GsBase::read().as_u64() as usize;

    FsBase::write(x86_64::VirtAddr::new(next.arch_context.fsbase as u64));
    GsBase::write(x86_64::VirtAddr::new(next.arch_context.gsbase as u64));

    prev.arch_context.fpu.save();
    next.arch_context.fpu.restore();
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
