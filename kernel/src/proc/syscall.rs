use core::hint::spin_loop;

use crate::{
    arch::{arch_disable_intr, arch_enable_intr, arch_yield, proc::arch_get_cpu_id},
    fs::fd::FILE_DESCRIPTOR_MANAGERS,
    syscall::Result,
};

use super::{
    ContextStatus, PosixOldUtsName,
    sched::{SCHEDULER, SCHEDULER_ENABLED, get_current_context},
};

pub fn sys_fork(regs_ptr: usize) -> Result<usize> {
    SCHEDULER_ENABLED.store(false, core::sync::atomic::Ordering::SeqCst);
    let res = get_current_context().read().do_fork(regs_ptr);
    SCHEDULER_ENABLED.store(true, core::sync::atomic::Ordering::SeqCst);
    res
}

pub fn sys_exit(code: usize) -> ! {
    let context = get_current_context();

    context.write().code = code;
    context.write().status = ContextStatus::Died;

    loop {
        arch_yield();
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
        if let Some(context) = SCHEDULER.lock().currents.get(&cpu_id) {
            if context.read().status == ContextStatus::Died {
                arch_disable_intr();

                FILE_DESCRIPTOR_MANAGERS
                    .lock()
                    .remove(&context.read().get_pid())
                    .unwrap();

                let idx = SCHEDULER.lock().currents.remove(&cpu_id);

                unsafe { core::ptr::write_volatile(status_ptr as *mut usize, context.read().code) };

                return Ok(pid);
            }

            arch_enable_intr();

            spin_loop();
        } else {
            return Err(crate::errno::Errno::ENOENT);
        }
    }
}

pub fn sys_uname(name: *mut PosixOldUtsName) -> Result<usize> {
    unsafe { *name = PosixOldUtsName::new() };
    Ok(0)
}
