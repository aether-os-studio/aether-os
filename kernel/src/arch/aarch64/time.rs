use core::hint::spin_loop;

use aarch64::regs::{CNTFRQ_EL0, CNTPCT_EL0, Readable};

use crate::arch::time::TimeArch;

pub struct AArch64TimeArch;

impl TimeArch for AArch64TimeArch {
    fn nano_time() -> u64 {
        let freq = CNTFRQ_EL0.get() as u64;
        let ticks = CNTPCT_EL0.get() as u64;
        let temp = ticks as u128 * 1000000000;
        (temp / freq as u128) as u64
    }

    fn delay(ns: u64) {
        let timeout = AArch64TimeArch::nano_time() as u64 + ns;
        while AArch64TimeArch::nano_time() < timeout {
            spin_loop();
        }
    }
}
