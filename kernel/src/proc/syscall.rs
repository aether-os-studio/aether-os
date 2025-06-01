use crate::syscall::Result;

use super::sched::{SCHEDULER_ENABLED, get_current_context};

pub fn sys_fork(regs_ptr: usize) -> Result<usize> {
    SCHEDULER_ENABLED.store(false, core::sync::atomic::Ordering::SeqCst);
    let res = get_current_context().read().do_fork(regs_ptr);
    SCHEDULER_ENABLED.store(true, core::sync::atomic::Ordering::SeqCst);
    res
}
