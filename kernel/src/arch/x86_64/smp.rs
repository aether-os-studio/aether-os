use core::hint::spin_loop;

use alloc::collections::btree_map::BTreeMap;
use limine::mp::Cpu;
use rmm::{Arch, PhysicalAddress, TableKind};
use spin::Mutex;

use crate::{
    arch::{
        CurrentIrqArch, CurrentRmmArch,
        drivers::apic::{APIC_INITIALIZED, LAPIC, LAPIC_TIMER_INITIAL},
        gdt::CpuInfo,
        init_sse,
        irq::IrqArch,
    },
    init::memory::KERNEL_PAGE_TABLE_PHYS,
    smp::{BSP_CPUARCHID, CPU_COUNT, CPUID_TO_ARCHID, MP_REQUEST},
    task::{
        get_current_task,
        sched::{SCHEDULERS, Scheduler},
    },
};

unsafe extern "C" {
    unsafe fn _ap_start(cpu: &Cpu) -> !;
}

pub static LAPICID_TO_CPUINFO: Mutex<BTreeMap<usize, CpuInfo>> = Mutex::new(BTreeMap::new());

pub fn get_lapicid() -> usize {
    unsafe { LAPIC.lock().as_mut().unwrap().id() as usize }
}

pub fn init() {
    let mp_response = MP_REQUEST.get_response().unwrap();
    BSP_CPUARCHID.store(
        mp_response.bsp_lapic_id() as usize,
        core::sync::atomic::Ordering::SeqCst,
    );
    CPU_COUNT.store(
        mp_response.cpus().len(),
        core::sync::atomic::Ordering::SeqCst,
    );
    for (i, cpu) in mp_response.cpus().iter().enumerate() {
        LAPICID_TO_CPUINFO
            .lock()
            .insert(cpu.lapic_id as usize, CpuInfo::default());
        SCHEDULERS
            .lock()
            .insert(cpu.lapic_id as usize, Scheduler::new());
        CPUID_TO_ARCHID.lock().insert(i, cpu.lapic_id as usize);
        if cpu.lapic_id == mp_response.bsp_lapic_id() {
            continue;
        }
        cpu.goto_address.write(_ap_start);
    }
    LAPICID_TO_CPUINFO
        .lock()
        .get_mut(&(mp_response.bsp_lapic_id() as usize))
        .unwrap()
        .init();
}

#[unsafe(no_mangle)]
extern "C" fn ap_kmain(cpu: &Cpu) -> ! {
    CurrentIrqArch::disable_global_irq();

    let physical_address =
        PhysicalAddress::new(KERNEL_PAGE_TABLE_PHYS.load(core::sync::atomic::Ordering::SeqCst));
    unsafe { CurrentRmmArch::set_table(TableKind::Kernel, physical_address) };

    init_sse();

    LAPICID_TO_CPUINFO
        .lock()
        .get_mut(&(cpu.lapic_id as usize))
        .unwrap()
        .init();
    crate::arch::x86_64::irq::init();

    while !APIC_INITIALIZED.load(core::sync::atomic::Ordering::SeqCst) {
        spin_loop();
    }

    let timer_initial = LAPIC_TIMER_INITIAL.load(core::sync::atomic::Ordering::SeqCst);

    if let Some(lapic) = LAPIC.lock().as_mut() {
        unsafe {
            lapic.enable();
            lapic.set_timer_initial(timer_initial);
        };
    }

    crate::arch::x86_64::syscall::init();

    while get_current_task().is_none() {
        spin_loop();
    }

    loop {
        CurrentIrqArch::enable_global_irq();
        spin_loop();
    }
}
