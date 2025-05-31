use x86_64::VirtAddr;

use super::{apic::LAPIC, mp::CPUS};

pub mod context;
pub mod syscall;

pub fn arch_get_cpu_id() -> u32 {
    unsafe { LAPIC.lock().id() }
}

pub fn arch_set_kernel_stack(stack: usize) {
    CPUS.write()
        .get_mut(arch_get_cpu_id())
        .set_ring0_rsp(VirtAddr::new(stack as u64));
}
