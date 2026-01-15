use core::hint::spin_loop;

use crate::arch::{drivers::hpet::HPET, time::TimeArch};

pub struct X8664TimeArch;

impl TimeArch for X8664TimeArch {
    fn nano_time() -> u64 {
        HPET.elapsed().as_nanos() as u64
    }

    fn delay(ns: u64) {
        let timeout = X8664TimeArch::nano_time() + ns;
        while X8664TimeArch::nano_time() < timeout {
            spin_loop();
        }
    }
}
