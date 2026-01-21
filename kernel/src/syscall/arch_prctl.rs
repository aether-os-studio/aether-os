use x86_64::{
    VirtAddr,
    registers::model_specific::{FsBase, GsBase},
};

use crate::{
    syscall::error::{ENOSYS, Error, Result},
    task::get_current_task,
};

pub const ARCH_SET_GS: usize = 0x1001;
pub const ARCH_SET_FS: usize = 0x1002;
pub const ARCH_GET_FS: usize = 0x1003;
pub const ARCH_GET_GS: usize = 0x1004;

pub fn sys_arch_prctl(option: usize, arg: usize) -> Result<usize> {
    let ret = match option {
        ARCH_GET_FS => FsBase::read().as_u64() as usize,
        ARCH_GET_GS => GsBase::read().as_u64() as usize,
        ARCH_SET_FS => {
            FsBase::write(VirtAddr::new(arg as u64));
            get_current_task().unwrap().write().arch_context.fsbase = arg;
            0
        }
        ARCH_SET_GS => 0,
        _ => -ENOSYS as usize,
    };
    Error::demux(ret)
}
