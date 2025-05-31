use crate::{errno::Errno, proc::sched::get_current_context};

pub const ARCH_SET_GS: usize = 0x1001;
pub const ARCH_SET_FS: usize = 0x1002;
pub const ARCH_GET_FS: usize = 0x1003;
pub const ARCH_GET_GS: usize = 0x1004;

pub fn sys_arch_prctl(cmd: usize, arg: usize) -> usize {
    match cmd {
        ARCH_SET_GS => {
            get_current_context().write().arch_mut().set_fs(arg);
            0
        }
        ARCH_SET_FS => 0,
        ARCH_GET_GS => 0,
        ARCH_GET_FS => get_current_context().read().arch().get_fs(),
        _ => Errno::EINVAL.to_posix_errno() as usize,
    }
}
