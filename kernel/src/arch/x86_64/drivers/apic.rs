use core::{
    sync::atomic::{AtomicBool, AtomicU32},
    time::Duration,
};

use acpi::sdt::madt::{Madt, MadtEntry};
use alloc::vec::Vec;
use rmm::{Arch, PageFlags, PhysicalAddress};
use spin::Mutex;
use x2apic::{
    ioapic::RedirectionTableEntry,
    lapic::{LocalApic, LocalApicBuilder, TimerMode},
};
use x86_64::instructions::port::Port;

use crate::{
    arch::{
        CurrentRmmArch,
        drivers::hpet::HPET,
        smp::get_lapicid,
        x86_64::irq::{INTERRUPT_INDEX_OFFSET, InterruptIndex},
    },
    consts::SCHED_HZ,
    drivers::acpi::ACPI_TABLES,
    init::memory::FRAME_ALLOCATOR,
    memory::mapper::KernelPageMapper,
};

pub struct IoApic {
    ioapic: x2apic::ioapic::IoApic,
    gsi_start: u32,
    count: usize,
}

impl IoApic {
    pub fn init(&mut self) {
        unsafe { self.ioapic.init(INTERRUPT_INDEX_OFFSET) };
    }

    pub fn map(&mut self, idx: u8, vector: u8) {
        let mut entry = RedirectionTableEntry::default();
        entry.set_dest(get_lapicid() as u8);
        entry.set_vector(vector);
        unsafe { self.ioapic.set_table_entry(idx, entry) };
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Override {
    bus_irq: u8,
    gsi: u32,
}

pub static LAPIC: Mutex<Option<LocalApic>> = Mutex::new(None);
static IOAPICS: Mutex<Vec<IoApic>> = Mutex::new(Vec::new());
static SRC_OVERRIDES: Mutex<Vec<Override>> = Mutex::new(Vec::new());

fn resolve(irq: u8) -> u32 {
    SRC_OVERRIDES
        .lock()
        .iter()
        .find(|over| over.bus_irq == irq)
        .map_or(u32::from(irq), |over| over.gsi)
}

fn use_ioapic<F>(gsi: u32, cb: F)
where
    F: FnOnce(&mut IoApic),
{
    if let Some(ioapic) = IOAPICS
        .lock()
        .iter_mut()
        .find(|apic| gsi >= apic.gsi_start && gsi < apic.gsi_start + apic.count as u32)
    {
        cb(ioapic)
    }
}

pub unsafe fn ioapic_add_entry(irq: u8, vector: u8) {
    let gsi = resolve(irq);
    use_ioapic(gsi, |ioapic| ioapic.map(irq, vector));
}

const TIMER_CALIBRATION_ITERATION: u32 = 5;

pub static APIC_INITIALIZED: AtomicBool = AtomicBool::new(false);
pub static LAPIC_TIMER_INITIAL: AtomicU32 = AtomicU32::new(0);

pub unsafe fn disable_pic() {
    Port::<u8>::new(0x21).write(0xff);
    Port::<u8>::new(0xa1).write(0xff);
}

pub unsafe fn calibrate_timer() {
    let mut lapic = LAPIC.lock();
    let lapic = lapic.as_mut().unwrap();
    let mut lapic_total_ticks = 0;

    for _ in 0..TIMER_CALIBRATION_ITERATION {
        let last_time = HPET.elapsed();
        lapic.set_timer_initial(u32::MAX);
        while HPET.elapsed() - last_time < Duration::from_millis(1) {}
        lapic_total_ticks += u32::MAX - lapic.timer_current();
    }

    let average_ticks_per_ms = lapic_total_ticks / TIMER_CALIBRATION_ITERATION;
    let calibrated_timer_initial = average_ticks_per_ms * 1000 / SCHED_HZ as u32;

    lapic.set_timer_mode(TimerMode::Periodic);
    lapic.set_timer_initial(calibrated_timer_initial);
    LAPIC_TIMER_INITIAL.store(
        calibrated_timer_initial,
        core::sync::atomic::Ordering::SeqCst,
    );
}

pub fn init() {
    let madt = ACPI_TABLES
        .lock()
        .as_mut()
        .expect("Failed to get acpi tables")
        .find_table::<Madt>()
        .expect("Failed to get madt");

    let lapic_physical = PhysicalAddress::new(madt.get().local_apic_address as usize);
    let lapic_virtual = unsafe { CurrentRmmArch::phys_to_virt(lapic_physical) };

    let mut frame_allocator = FRAME_ALLOCATOR.lock();
    let mut mapper = unsafe {
        KernelPageMapper::<CurrentRmmArch, _>::current(
            rmm::TableKind::Kernel,
            &mut *frame_allocator,
        )
    };

    unsafe { mapper.map_phys(lapic_virtual, lapic_physical, PageFlags::new().write(true)) };

    let mut lapic = LocalApicBuilder::new()
        .timer_vector(InterruptIndex::Timer as usize)
        .timer_mode(TimerMode::OneShot)
        .timer_initial(0)
        .error_vector(InterruptIndex::ApicError as usize)
        .spurious_vector(InterruptIndex::ApicSpurious as usize)
        .set_xapic_base(lapic_virtual.data() as u64)
        .build()
        .unwrap_or_else(|err| panic!("Failed to build local APIC: {:#?}", err));

    unsafe {
        disable_pic();
        lapic.enable()
    };

    *LAPIC.lock() = Some(lapic);

    for entry in madt.get().entries() {
        match entry {
            MadtEntry::IoApic(ioapic_entry) => {
                let ioapic_physical = PhysicalAddress::new(ioapic_entry.io_apic_address as usize);
                let ioapic_virtual = unsafe { CurrentRmmArch::phys_to_virt(ioapic_physical) };
                unsafe {
                    mapper.map_phys(
                        ioapic_virtual,
                        ioapic_physical,
                        PageFlags::new().write(true),
                    )
                };
                let mut ioapic =
                    unsafe { x2apic::ioapic::IoApic::new(ioapic_virtual.data() as u64) };
                let count = unsafe { ioapic.max_table_entry() } as usize;
                let mut ioapic = IoApic {
                    ioapic,
                    gsi_start: ioapic_entry.global_system_interrupt_base,
                    count,
                };
                ioapic.init();
                IOAPICS.lock().push(ioapic);
            }
            MadtEntry::InterruptSourceOverride(iso_entry) => {
                let src_override = Override {
                    bus_irq: iso_entry.irq,
                    gsi: iso_entry.global_system_interrupt,
                };
                SRC_OVERRIDES.lock().push(src_override);
            }
            _ => {}
        }
    }

    drop(frame_allocator);

    unsafe { calibrate_timer() };

    APIC_INITIALIZED.store(true, core::sync::atomic::Ordering::SeqCst);
}
