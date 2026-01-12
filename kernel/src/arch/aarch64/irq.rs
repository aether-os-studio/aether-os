use core::arch::{asm, global_asm};

use aarch64::regs::{ESR_EL1, Readable, VBAR_EL1, Writeable};

use crate::arch::irq::{IrqArch, IrqRegsArch};

global_asm!(include_str!("vector.S"));

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Ptrace {
    pc: u64,
    cpsr: u64,
    sp_el0: u64,
    x30: u64,
    x28: u64,
    x29: u64,
    x26: u64,
    x27: u64,
    x24: u64,
    x25: u64,
    x22: u64,
    x23: u64,
    x20: u64,
    x21: u64,
    x18: u64,
    x19: u64,
    x16: u64,
    x17: u64,
    x14: u64,
    x15: u64,
    x12: u64,
    x13: u64,
    x10: u64,
    x11: u64,
    x8: u64,
    x9: u64,
    x6: u64,
    x7: u64,
    x4: u64,
    x5: u64,
    x2: u64,
    x3: u64,
    x0: u64,
    x1: u64,
}

impl IrqRegsArch for Ptrace {
    fn get_ip(&self) -> u64 {
        self.pc
    }

    fn set_ip(&mut self, ip: u64) {
        self.pc = ip;
    }

    fn get_sp(&self) -> u64 {
        self.sp_el0
    }

    fn set_sp(&mut self, sp: u64) {
        self.sp_el0 = sp;
    }

    fn get_ret_value(&self) -> u64 {
        self.x0
    }

    fn set_ret_value(&mut self, ret_value: u64) {
        self.x0 = ret_value
    }

    fn get_ret_address(&self) -> u64 {
        self.x30
    }

    fn set_ret_address(&mut self, ret_address: u64) {
        self.x30 = ret_address
    }

    fn get_syscall_idx(&self) -> u64 {
        self.x8
    }

    fn get_syscall_args(&self) -> (u64, u64, u64, u64, u64, u64) {
        (self.x0, self.x1, self.x2, self.x3, self.x4, self.x5)
    }
}

pub struct AArch64IrqArch;

impl IrqArch for AArch64IrqArch {
    fn enable_global_irq() {
        unsafe { asm!("msr daifclr, #3", "isb", options(nomem, nostack)) };
    }

    fn disable_global_irq() {
        unsafe { asm!("msr daifset, #3", "isb", options(nomem, nostack)) };
    }
}

unsafe extern "C" {
    unsafe fn exception_vectors();
}

#[unsafe(no_mangle)]
extern "C" fn handle_irq_el1_spx(ctx: &mut Ptrace) {
    panic!("IRQ at PC: 0x{:016x}", ctx.pc);
}

#[unsafe(no_mangle)]
extern "C" fn handle_sync_el1_spx(ctx: &mut Ptrace) {
    error!("Sync exception at PC: 0x{:016x}", ctx.pc);
    panic!("ESR_EL1: 0x{:016x}", ESR_EL1.get());
}

#[unsafe(no_mangle)]
extern "C" fn handle_fiq_el1_spx(ctx: &mut Ptrace) {
    panic!("FIQ at PC: 0x{:016x}", ctx.pc);
}

#[unsafe(no_mangle)]
extern "C" fn handle_serror_el1_spx(ctx: &mut Ptrace) {
    panic!("SError at PC: 0x{:016x}", ctx.pc);
}

#[unsafe(no_mangle)]
extern "C" fn handle_sync_el1_sp0(ctx: &mut Ptrace) {
    error!("Sync exception at PC: 0x{:016x}", ctx.pc);
    panic!("ESR_EL1: 0x{:016x}", ESR_EL1.get());
}
#[unsafe(no_mangle)]
extern "C" fn handle_irq_el1_sp0(ctx: &mut Ptrace) {
    panic!("IRQ at PC: 0x{:016x}", ctx.pc);
}

#[unsafe(no_mangle)]
extern "C" fn handle_fiq_el1_sp0(ctx: &mut Ptrace) {
    panic!("FIQ at PC: 0x{:016x}", ctx.pc);
}
#[unsafe(no_mangle)]
extern "C" fn handle_serror_el1_sp0(ctx: &mut Ptrace) {
    panic!("SError at PC: 0x{:016x}", ctx.pc);
}
#[unsafe(no_mangle)]
extern "C" fn handle_sync_el0_64(ctx: &mut Ptrace) {
    error!("Sync exception at PC: 0x{:016x}", ctx.pc);
    panic!("ESR_EL1: 0x{:016x}", ESR_EL1.get());
}
#[unsafe(no_mangle)]
extern "C" fn handle_irq_el0_64(ctx: &mut Ptrace) {
    panic!("IRQ at PC: 0x{:016x}", ctx.pc);
}
#[unsafe(no_mangle)]
extern "C" fn handle_fiq_el0_64(ctx: &mut Ptrace) {
    panic!("FIQ at PC: 0x{:016x}", ctx.pc);
}
#[unsafe(no_mangle)]
extern "C" fn handle_serror_el0_64(ctx: &mut Ptrace) {
    panic!("SError at PC: 0x{:016x}", ctx.pc);
}
#[unsafe(no_mangle)]
extern "C" fn handle_sync_el0_32(_ctx: &mut Ptrace) {}
#[unsafe(no_mangle)]
extern "C" fn handle_irq_el0_32(_ctx: &mut Ptrace) {}
#[unsafe(no_mangle)]
extern "C" fn handle_fiq_el0_32(_ctx: &mut Ptrace) {}
#[unsafe(no_mangle)]
extern "C" fn handle_serror_el0_32(_ctx: &mut Ptrace) {}

pub fn init() {
    VBAR_EL1.set(exception_vectors as *const () as u64);
}
