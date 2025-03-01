use acpi::HpetInfo;
use rmm::{Arch, PageFlags, PhysicalAddress, VirtualAddress};

use crate::memory::{mapper::KernelMapper, RmmA};

const LEG_RT_CNF: u64 = 2;
const ENABLE_CNF: u64 = 1;
const TN_VAL_SET_CNF: u64 = 0x40;
const TN_TYPE_CNF: u64 = 0x08;
const TN_INT_ENB_CNF: u64 = 0x04;
pub(crate) const CAPABILITY_OFFSET: usize = 0x00;
const GENERAL_CONFIG_OFFSET: usize = 0x10;
const GENERAL_INTERRUPT_OFFSET: usize = 0x20;
pub(crate) const MAIN_COUNTER_OFFSET: usize = 0xF0;
// const NUM_TIMER_CAP_MASK: u64 = 0x0f00;
const LEG_RT_CAP: u64 = 0x8000;
const T0_CONFIG_CAPABILITY_OFFSET: usize = 0x100;
pub(crate) const T0_COMPARATOR_OFFSET: usize = 0x108;
const PER_INT_CAP: u64 = 0x10;

// 1 / (1.193182 MHz) = 838,095,110 femtoseconds ~= 838.095 ns
pub const PERIOD_FS: u128 = 838_095_110;
// 4847 / (1.193182 MHz) = 4,062,247 ns ~= 4.1 ms or 246 Hz
pub const CHAN0_DIVISOR: u16 = 4847;
// Calculated interrupt period in nanoseconds based on divisor and period
pub const RATE: u128 = (CHAN0_DIVISOR as u128 * PERIOD_FS) / 1_000_000;

pub fn read_u64(addr: VirtualAddress, offset: usize) -> u64 {
    unsafe { core::ptr::read_volatile(addr.add(offset).data() as *const u64) }
}

pub fn write_u64(addr: VirtualAddress, offset: usize, value: u64) {
    unsafe { core::ptr::write_volatile(addr.add(offset).data() as *mut u64, value) }
}

pub unsafe fn init(active_table: &mut KernelMapper, hpet: HpetInfo) -> bool {
    let phys_addr = PhysicalAddress::new(hpet.base_address);
    let virt_addr = RmmA::phys_to_virt(phys_addr);

    active_table
        .get_mut()
        .unwrap()
        .map_phys(virt_addr, phys_addr, PageFlags::new().write(true))
        .unwrap()
        .flush();

    // Disable HPET
    {
        let mut config_word = read_u64(virt_addr, GENERAL_CONFIG_OFFSET);
        config_word &= !(LEG_RT_CNF | ENABLE_CNF);
        write_u64(virt_addr, GENERAL_CONFIG_OFFSET, config_word);
    }
    let capability = read_u64(virt_addr, CAPABILITY_OFFSET);
    if capability & LEG_RT_CAP == 0 {
        log::warn!("HPET missing capability LEG_RT_CAP");
        return false;
    }
    let period_fs = capability >> 32;
    let divisor = (RATE as u64 * 1_000_000) / period_fs;
    let t0_capabilities = read_u64(virt_addr, T0_CONFIG_CAPABILITY_OFFSET);
    if t0_capabilities & PER_INT_CAP == 0 {
        log::warn!("HPET T0 missing capability PER_INT_CAP");
        return false;
    }
    let counter = read_u64(virt_addr, MAIN_COUNTER_OFFSET);
    let t0_config_word: u64 = TN_VAL_SET_CNF | TN_TYPE_CNF | TN_INT_ENB_CNF;
    write_u64(virt_addr, T0_CONFIG_CAPABILITY_OFFSET, t0_config_word);
    // set accumulator value
    write_u64(virt_addr, T0_COMPARATOR_OFFSET, counter + divisor);
    // set interval
    write_u64(virt_addr, T0_COMPARATOR_OFFSET, divisor);
    // Enable interrupts from the HPET
    {
        let mut config_word: u64 = read_u64(virt_addr, GENERAL_CONFIG_OFFSET);
        config_word |= LEG_RT_CNF | ENABLE_CNF;
        write_u64(virt_addr, GENERAL_CONFIG_OFFSET, config_word);
    }
    log::debug!("HPET After Init");
    debug(virt_addr);
    true
}

pub unsafe fn debug(virt_addr: VirtualAddress) {
    let capability = read_u64(virt_addr, CAPABILITY_OFFSET);
    {
        log::debug!("  caps: {:#x}", capability);
        log::debug!("    clock period: {}", (capability >> 32) as u32);
        log::debug!("    ID: {:#x}", (capability >> 16) as u16);
        log::debug!("    LEG_RT_CAP: {}", capability & (1 << 15) == (1 << 15));
        log::debug!(
            "    COUNT_SIZE_CAP: {}",
            capability & (1 << 13) == (1 << 13)
        );
        log::debug!("    timers: {}", (capability >> 8) as u8 & 0x1F);
        log::debug!("    revision: {}", capability as u8);
    }
    let config_word = read_u64(virt_addr, GENERAL_CONFIG_OFFSET);
    log::debug!("  config: {:#x}", config_word);
    let interrupt_status = read_u64(virt_addr, GENERAL_INTERRUPT_OFFSET);
    log::debug!("  interrupt status: {:#x}", interrupt_status);
    let counter = read_u64(virt_addr, MAIN_COUNTER_OFFSET);
    log::debug!("  counter: {:#x}", counter);
    let t0_capabilities = read_u64(virt_addr, T0_CONFIG_CAPABILITY_OFFSET);
    log::debug!("  T0 caps: {:#x}", t0_capabilities);
    log::debug!(
        "    interrupt routing: {:#x}",
        (t0_capabilities >> 32) as u32
    );
    log::debug!("    flags: {:#x}", t0_capabilities as u16);
    let t0_comparator = read_u64(virt_addr, T0_COMPARATOR_OFFSET);
    log::debug!("  T0 comparator: {:#x}", t0_comparator);
}
