use core::hint::spin_loop;

use alloc::collections::btree_map::BTreeMap;
use spin::RwLock;

use crate::{
    arch::{arch_disable_intr, arch_enable_intr, proc::arch_get_cpu_id},
    fs::fd::FILE_DESCRIPTOR_MANAGERS,
    proc::sched::SCHEDULER,
    syscall::Result,
};

use super::{
    PosixOldUtsName,
    sched::{SCHEDULER_ENABLED, get_current_context},
};

pub fn sys_fork(regs_ptr: usize) -> Result<usize> {
    SCHEDULER_ENABLED.store(false, core::sync::atomic::Ordering::SeqCst);
    let res = get_current_context().read().do_fork(regs_ptr);
    SCHEDULER_ENABLED.store(true, core::sync::atomic::Ordering::SeqCst);
    res
}

pub static EXIT_CODE_MAP: RwLock<BTreeMap<usize, usize>> = RwLock::new(BTreeMap::new());

pub fn sys_exit(code: usize) -> ! {
    let context = get_current_context();

    EXIT_CODE_MAP.write().insert(context.read().get_pid(), code);

    FILE_DESCRIPTOR_MANAGERS
        .lock()
        .remove(&context.read().get_pid())
        .unwrap();

    SCHEDULER
        .lock()
        .currents
        .remove(&arch_get_cpu_id())
        .unwrap();

    loop {
        arch_enable_intr();
        spin_loop();
    }
}

pub fn sys_wait4(
    pid: usize,
    status_ptr: usize,
    options: usize,
    rusage_ptr: usize,
) -> Result<usize> {
    let cpu_id = arch_get_cpu_id();

    loop {
        if let Some(exit_code) = EXIT_CODE_MAP.write().remove(&pid) {
            arch_disable_intr();

            unsafe { core::ptr::write_volatile(status_ptr as *mut u32, exit_code as u32) };

            return Ok(pid);
        }

        arch_enable_intr();

        spin_loop();
    }
}

pub fn sys_uname(name: *mut PosixOldUtsName) -> Result<usize> {
    unsafe { *name = PosixOldUtsName::new() };
    Ok(0)
}
