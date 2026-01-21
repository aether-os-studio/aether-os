use crate::{
    arch::{Ptrace, SYS_ARCH_PRCTL, irq::IrqRegsArch},
    syscall::{
        arch_prctl::sys_arch_prctl,
        error::{ENOSYS, Error},
    },
};

pub mod error;
pub mod usercopy;

#[cfg(target_arch = "x86_64")]
mod arch_prctl;

pub extern "C" fn syscall_handler(regs: *mut Ptrace) {
    let regs = unsafe { regs.as_mut_unchecked() };

    let idx = regs.get_syscall_idx() as usize;
    let (arg1, arg2, arg3, arg4, arg5, arg6) = regs.get_syscall_args();
    let (arg1, arg2, arg3, arg4, arg5, arg6) = (
        arg1 as usize,
        arg2 as usize,
        arg3 as usize,
        arg4 as usize,
        arg5 as usize,
        arg6 as usize,
    );

    let ret = match idx {
        #[cfg(target_arch = "x86_64")]
        SYS_ARCH_PRCTL => sys_arch_prctl(arg1, arg2),
        _ => {
            warn!("Syscall {} not implemented", idx);
            Error::demux(-ENOSYS as usize)
        }
    };

    regs.set_ret_value(Error::mux(ret) as u64);
}
