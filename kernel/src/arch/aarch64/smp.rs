use core::{arch::asm, hint::spin_loop};

use limine::mp::Cpu;
use rmm::{Arch, PhysicalAddress, TableKind};

use crate::{
    arch::{
        CurrentIrqArch, CurrentRmmArch,
        drivers::gic::{GIC_IRQ_CONTROLLER, GIC_IRQ_INITIALIZED, Gic},
        irq::IrqArch,
    },
    init::memory::KERNEL_PAGE_TABLE_PHYS,
    smp::{BSP_CPUARCHID, CPU_COUNT, CPUID_TO_ARCHID, MP_REQUEST, get_cpuid_by_archid},
    task::sched::{SCHEDULERS, Scheduler},
};

unsafe extern "C" {
    unsafe fn _ap_start(cpu: &Cpu) -> !;
}
pub fn get_mpidr() -> usize {
    let mut val: u64 = 0;
    unsafe { asm!("mrs {0}, mpidr_el1", out(reg) val) };
    val as usize & !0x80000000
}

pub fn init() {
    if let Some(mp_response) = MP_REQUEST.get_response() {
        BSP_CPUARCHID.store(
            mp_response.bsp_mpidr() as usize,
            core::sync::atomic::Ordering::SeqCst,
        );
        CPU_COUNT.store(
            mp_response.cpus().len(),
            core::sync::atomic::Ordering::SeqCst,
        );
        for (i, cpu) in mp_response.cpus().iter().enumerate() {
            SCHEDULERS
                .lock()
                .insert(cpu.mpidr as usize, Scheduler::new());
            CPUID_TO_ARCHID.lock().insert(i, cpu.mpidr as usize);
            if cpu.mpidr == mp_response.bsp_mpidr() {
                continue;
            }
            cpu.goto_address.write(_ap_start);
        }
    }
}

#[unsafe(no_mangle)]
extern "C" fn ap_kmain(cpu: &Cpu) -> ! {
    CurrentIrqArch::disable_global_irq();

    crate::arch::rmm::init();

    let physical_address =
        PhysicalAddress::new(KERNEL_PAGE_TABLE_PHYS.load(core::sync::atomic::Ordering::SeqCst));
    unsafe { CurrentRmmArch::set_table(TableKind::Kernel, physical_address) };

    while !GIC_IRQ_INITIALIZED.load(core::sync::atomic::Ordering::SeqCst) {
        spin_loop();
    }
    if let Some(gic) = GIC_IRQ_CONTROLLER.lock().as_mut()
        && let Gic::V3(v3) = gic
    {
        v3.setup(get_cpuid_by_archid(cpu.mpidr as usize));
    }

    crate::arch::drivers::timer::init();

    loop {
        CurrentIrqArch::enable_global_irq();
        spin_loop();
    }
}
