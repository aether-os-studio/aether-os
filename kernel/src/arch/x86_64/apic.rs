use core::sync::atomic::AtomicU32;

use acpi::InterruptModel;
use rmm::{Arch, PageFlags, PhysicalAddress};
use spin::{Lazy, Mutex, MutexGuard};
use x2apic::ioapic::{IrqMode, RedirectionTableEntry};
use x2apic::lapic::{LocalApicBuilder, TimerMode};
use x2apic::{ioapic::IoApic, lapic::LocalApic};

use crate::arch::CurrentMMArch;
use crate::arch::hpet::HPET;

use crate::memory::KERNEL_PAGE_TABLE;
use crate::serial_println;

use super::acpi::ACPI;
use super::interrupts::InterruptIndex;

pub struct LockedLocalApic(Mutex<LocalApic>);

impl LockedLocalApic {
    pub fn new(addr: usize) -> LockedLocalApic {
        let phys = PhysicalAddress::new(addr);
        let virt = unsafe { CurrentMMArch::phys_to_virt(phys) };
        unsafe {
            let result =
                KERNEL_PAGE_TABLE
                    .lock()
                    .map_phys(virt, phys, PageFlags::new().write(true));

            if let Some(flusher) = result {
                flusher.flush();
            }
        }

        let lapic = LocalApicBuilder::new()
            .timer_vector(InterruptIndex::Timer as usize)
            .timer_mode(TimerMode::OneShot)
            .timer_initial(0)
            .error_vector(InterruptIndex::ApicError as usize)
            .spurious_vector(InterruptIndex::ApicSpurious as usize)
            .set_xapic_base(virt.data() as u64)
            .build()
            .unwrap_or_else(|err| panic!("Failed to build local APIC: {:#?}", err));

        Self(Mutex::new(lapic))
    }

    pub fn lock(&self) -> MutexGuard<'_, LocalApic> {
        self.0.lock()
    }
}

unsafe impl Send for LockedLocalApic {}
unsafe impl Sync for LockedLocalApic {}

pub struct LockedIoApic(Mutex<IoApic>);

impl LockedIoApic {
    pub fn new(addr: usize) -> Self {
        let phys = PhysicalAddress::new(addr);
        let virt = unsafe { CurrentMMArch::phys_to_virt(phys) };
        unsafe {
            let result =
                KERNEL_PAGE_TABLE
                    .lock()
                    .map_phys(virt, phys, PageFlags::new().write(true));

            if let Some(flusher) = result {
                flusher.flush();
            }
        }

        Self(Mutex::new(unsafe { IoApic::new(virt.data() as u64) }))
    }

    pub fn lock(&self) -> MutexGuard<'_, IoApic> {
        self.0.lock()
    }
}

unsafe impl Send for LockedIoApic {}
unsafe impl Sync for LockedIoApic {}

pub static LAPIC: Lazy<LockedLocalApic> = Lazy::new(|| {
    let acpi_tables = ACPI.get().unwrap();
    let acpi_tables = acpi_tables.lock();
    let info = acpi_tables.platform_info().unwrap().interrupt_model;
    let apic = match info {
        InterruptModel::Apic(apic) => apic,
        _ => panic!(),
    };
    LockedLocalApic::new(apic.local_apic_address as usize)
});

pub static IOAPIC: Lazy<LockedIoApic> = Lazy::new(|| {
    let acpi_tables = ACPI.get().unwrap();
    let acpi_tables = acpi_tables.lock();
    let info = acpi_tables.platform_info().unwrap().interrupt_model;
    let apic = match info {
        InterruptModel::Apic(apic) => apic,
        _ => panic!(),
    };
    LockedIoApic::new(apic.io_apics[0].address as usize)
});

unsafe fn ioapic_add_entry(irq: InterruptIndex, vector: InterruptIndex) {
    let lapic = LAPIC.lock();
    let mut ioapic = IOAPIC.lock();
    let mut entry = RedirectionTableEntry::default();
    entry.set_mode(IrqMode::Fixed);
    entry.set_dest(lapic.id() as u8);
    entry.set_vector(vector as u8);
    ioapic.set_table_entry(irq as u8, entry);
    ioapic.enable_irq(irq as u8);
}

pub const TIMER_FREQUENCY_HZ: u32 = 250;

pub static CALIBRATED_LAPIC_TIMER_INITIAL: AtomicU32 = AtomicU32::new(0);

pub unsafe fn calibrate_timer() {
    let mut lapic = LAPIC.0.lock();

    let mut lapic_total_ticks = 0;
    let hpet_clock_speed = HPET.clock_speed() as u64;
    let hpet_tick_per_ms = 1_000_000_000_000 / hpet_clock_speed;

    for _ in 0..5 {
        let next_ms = HPET.get_counter() + hpet_tick_per_ms;
        lapic.set_timer_initial(!0);
        while HPET.get_counter() < next_ms {}
        lapic_total_ticks += !0 - lapic.timer_current();
    }

    let average_clock_per_ms = lapic_total_ticks / 5;
    serial_println!("Calibrated clock per ms: {}", average_clock_per_ms);

    lapic.set_timer_mode(TimerMode::Periodic);
    let initial = average_clock_per_ms * 1000 / TIMER_FREQUENCY_HZ;
    lapic.set_timer_initial(initial);
    CALIBRATED_LAPIC_TIMER_INITIAL.store(initial, core::sync::atomic::Ordering::SeqCst);
}

fn init_apic() {
    Lazy::force(&LAPIC);
}

fn init_ioapic() {
    Lazy::force(&IOAPIC);
}

pub fn end_of_interrupt() {
    unsafe { LAPIC.0.lock().end_of_interrupt() };
}

pub fn init() {
    init_apic();
    init_ioapic();
    unsafe { calibrate_timer() };
    unsafe { LAPIC.lock().enable() };
}
