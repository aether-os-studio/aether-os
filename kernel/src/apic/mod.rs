use core::sync::atomic::Ordering;

pub mod hpet;
mod ioapic;
mod lapic;

use crate::pctable::idt::InterruptIndex;
pub use ioapic::{IrqVector, ioapic_add_entry};
pub use lapic::{APIC_INIT, LAPIC, LAPIC_TIMER_INITIAL};
pub use lapic::{disable_pic, end_of_interrupt};

pub fn init() {
    unsafe {
        disable_pic();
        lapic::calibrate_timer();

        ioapic_add_entry(IrqVector::Keyboard, InterruptIndex::Keyboard);
        ioapic_add_entry(IrqVector::Mouse, InterruptIndex::Mouse);
        ioapic_add_entry(IrqVector::HpetTimer, InterruptIndex::HpetTimer);
    };

    APIC_INIT.store(true, Ordering::SeqCst);
    info!("APIC initialized successfully!");
}
