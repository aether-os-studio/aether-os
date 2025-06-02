use crate::{
    arch::nr::{SigAction, SignalStack},
    errno::Errno,
    syscall::{Result, check_user_overflows},
};

use super::sched::get_current_context;

pub fn sys_rt_sigaction(
    signum: usize,
    act: *const SigAction,
    oldact: *mut SigAction,
    sigsetsize: usize,
) -> Result<usize> {
    // 验证信号有效性
    if signum > 64 || sigsetsize != 8 {
        return Err(Errno::EINVAL);
    }

    let ctx = get_current_context();
    let mut ctx_lock = ctx.write();

    // 保存旧的处理函数
    if !oldact.is_null() && !check_user_overflows(oldact as usize, size_of::<SigAction>()) {
        let old = ctx_lock.actions.get(&signum);
        unsafe { *oldact = old.cloned().unwrap_or_default() };
    }

    // 设置新的处理函数
    if !act.is_null() && !check_user_overflows(act as usize, size_of::<SigAction>()) {
        let new_action = unsafe { &*act };
        ctx_lock.actions.insert(signum, new_action.clone());
    }

    Ok(0)
}

pub const SS_ONSTACK: i32 = 1;
pub const SS_DISABLE: i32 = 2;
pub const MINSIGSTKSZ: usize = 2048;

pub fn sys_sigaltstack(ss: *const SignalStack, old_ss: *mut SignalStack) -> Result<usize> {
    // 保存旧栈
    if !old_ss.is_null() && !check_user_overflows(old_ss as usize, size_of::<SignalStack>()) {
        let old = get_current_context()
            .read()
            .signal_stack
            .map(|s| s)
            .unwrap_or_default();
        unsafe { *old_ss = old };
    }

    // 设置新栈
    if !ss.is_null() && !check_user_overflows(ss as usize, size_of::<SignalStack>()) {
        let new_stack = unsafe { *ss };
        get_current_context().write().signal_stack = Some(new_stack);
    }

    Ok(0)
}

pub const SIG_BLOCK: usize = 0;
pub const SIG_UNBLOCK: usize = 1;
pub const SIG_SETMASK: usize = 2;

pub fn sys_rt_sigprocmask(
    how: usize,
    set: *const u64,
    oldset: *mut u64,
    sigsetsize: usize,
) -> Result<usize> {
    // 验证信号集大小
    if sigsetsize < 8 {
        return Err(Errno::EINVAL);
    }

    let ctx = get_current_context();
    let mut ctx_lock = ctx.write();

    // 保存旧信号屏蔽
    if !oldset.is_null() {
        let dest = unsafe { core::slice::from_raw_parts_mut(oldset, 1) };
        dest[0] = ctx_lock.blocked;
    }

    // 设置新信号屏蔽
    if !set.is_null() {
        let src = unsafe { core::slice::from_raw_parts(set, 1) };
        match how {
            SIG_BLOCK => ctx_lock.blocked |= src[0],
            SIG_UNBLOCK => ctx_lock.blocked &= !src[0],
            SIG_SETMASK => ctx_lock.blocked = src[0],
            _ => return Err(Errno::EINVAL),
        }
        // 强制允许SIGKILL和SIGSTOP
        ctx_lock.blocked &= !((1 << 9) | (1 << 19));
    }

    Ok(0)
}
