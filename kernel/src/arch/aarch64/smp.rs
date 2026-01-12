use core::{arch::asm, hint::spin_loop};

use alloc::collections::btree_map::BTreeMap;
use limine::mp::Cpu;
use rmm::{Arch, PhysicalAddress, TableKind};
use spin::Mutex;

use crate::{
    arch::{
        CurrentIrqArch, CurrentRmmArch,
        drivers::gic::{GIC_IRQ_CONTROLLER, GIC_IRQ_INITIALIZED, Gic},
        irq::IrqArch,
    },
    init::memory::KERNEL_PAGE_TABLE_PHYS,
    smp::{BSP_CPUARCHID, CPU_COUNT, MP_REQUEST},
};

unsafe extern "C" {
    unsafe fn _ap_start(cpu: &Cpu) -> !;
}

pub static MPIDR_TO_CPUID: Mutex<BTreeMap<usize, usize>> = Mutex::new(BTreeMap::new());

pub fn get_mpidr() -> usize {
    let mut val: u64 = 0;
    unsafe { asm!("mrs {0}, mpidr_el1", out(reg) val) };
    val as usize & !0x80000000
}

pub fn get_cpuid_by_archid(archid: usize) -> usize {
    *MPIDR_TO_CPUID.lock().get(&archid).unwrap()
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
        for cpu in mp_response.cpus() {
            MPIDR_TO_CPUID
                .lock()
                .insert(cpu.mpidr as usize, cpu.id as usize);
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
    if let Some(gic) = GIC_IRQ_CONTROLLER.lock().as_mut() {
        if let Gic::V3(v3) = gic {
            v3.setup(get_cpuid_by_archid(cpu.mpidr as usize));
        }
    }

    crate::arch::drivers::timer::init();

    loop {
        CurrentIrqArch::enable_global_irq();
        spin_loop();
    }
}
