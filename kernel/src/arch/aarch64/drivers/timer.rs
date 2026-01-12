use aarch64::regs::{CNTFRQ_EL0, CNTP_CTL_EL0, CNTP_TVAL_EL0, CNTPCT_EL0, Readable, Writeable};

use crate::{
    arch::{CurrentIrqArch, irq::IrqArch},
    consts::SCHED_HZ,
};

pub fn read_cntpct() -> u64 {
    CNTPCT_EL0.get()
}

pub fn read_cntfrq() -> u64 {
    CNTFRQ_EL0.get()
}

pub fn read_cntp_ctl() -> u64 {
    CNTP_CTL_EL0.get()
}

pub fn write_cntp_ctl(val: u64) {
    CNTP_CTL_EL0.set(val)
}

pub fn write_cntp_tval(val: u64) {
    CNTP_TVAL_EL0.set(val);
}

pub fn timer_set_next_tick_ns(ns: u64) {
    let freq = read_cntfrq();
    let temp = ns as u128 * freq as u128;
    let delta_ticks = (temp / 1000000000) as u64;
    write_cntp_ctl(1);
    write_cntp_tval(delta_ticks);
}

pub fn init() {
    CurrentIrqArch::enable_global_irq();
    timer_set_next_tick_ns(1000000000 / SCHED_HZ as u64);
}
