use alloc::sync::Arc;
use asc::nr::*;
use error::SystemError;
use x86_64::VirtAddr;
use x86_64::registers::model_specific::{Efer, EferFlags};
use x86_64::registers::model_specific::{LStar, SFMask, Star};
use x86_64::registers::rflags::RFlags;
use x86_64::structures::paging::Translate;
use x86_64::structures::tss::TaskStateSegment;

use crate::apic::LAPIC;
use crate::context::context::Context;
use crate::context::scheduler::SCHEDULER;
use crate::memory::ref_current_page_table;
use crate::pctable::gdt::Selectors;
use crate::pctable::idt::InterruptIndex;
use crate::smp::CPUS;

pub fn init() {
    SFMask::write(RFlags::INTERRUPT_FLAG);
    LStar::write(VirtAddr::from_ptr(syscall_handler as *const ()));

    let (kernel_code, kernel_data) = Selectors::get_kernel_segments();
    let (user_code, user_data) = Selectors::get_user_segments();
    Star::write(user_code, user_data, kernel_code, kernel_data).unwrap();

    unsafe {
        Efer::write(Efer::read() | EferFlags::SYSTEM_CALL_EXTENSIONS);
    }
}

#[naked]
extern "C" fn syscall_handler() {
    unsafe {
        core::arch::naked_asm!(
            "sub rsp, 0x28",
            crate::push_context!(),
            "mov rdi, rsp",
            "call {syscall_matcher}",
            "mov rsp, rax",
            crate::pop_context!(),
            "add rsp, 0x28",
            "sysretq",
            syscall_matcher = sym syscall_matcher,
        );
    }
}

#[allow(unused_variables)]
pub extern "C" fn syscall_matcher(context: &mut Context) -> VirtAddr {
    context.rip = context.rcx;
    context.rflags = context.r11;
    context.rsp = (context as *mut Context).wrapping_add(1) as usize;
    let (code, data) = Selectors::get_user_segments();
    context.cs = code.0 as usize;
    context.ss = data.0 as usize;

    let syscall_index: usize = context.rax;
    let arg1 = context.rdi;
    let arg2 = context.rsi;
    let arg3 = context.rdx;
    let arg4 = context.r10;
    let arg5 = context.r8;
    let arg6 = context.r9;

    let result = match syscall_index {
        SYS_PUTSTRING => {
            let str =
                str::from_utf8(unsafe { core::slice::from_raw_parts(arg1 as *const u8, arg2) });
            if let Ok(str) = str {
                debug!("{}", str);
                context.rax = 0;
                return context.address();
            }
            SystemError::ENOENT.to_posix_errno() as isize
        }
        SYS_FORK => do_fork(context, false),
        SYS_VFORK => do_fork(context, true),
        SYS_EXIT => sys_exit(arg1),
        SYS_IOPL => {
            let allowed = arg1 >= 3;

            let offset = if allowed {
                u16::try_from(size_of::<TaskStateSegment>()).unwrap()
            } else {
                0xFFFF
            };

            let mut cpus = CPUS.write();
            cpus.get_mut(unsafe { LAPIC.lock().id() })
                .set_iomap_base(offset);

            0
        }
        SYS_VIRTTOPHYS => {
            let vaddr = arg1;
            if let Some(paddr) =
                ref_current_page_table().translate_addr(VirtAddr::new(vaddr as u64))
            {
                context.rax = paddr.as_u64() as usize;
                return context.address();
            }

            SystemError::ENOENT.to_posix_errno() as isize
        }
        _ => SystemError::ENOSYS.to_posix_errno() as isize,
    };

    context.rax = result as usize;

    context.address()
}

pub fn do_fork(context: &mut Context, vfork: bool) -> isize {
    let current = SCHEDULER.lock().current();
    if let Some(current) = current.upgrade() {
        return current
            .read()
            .process
            .upgrade()
            .unwrap()
            .read()
            .do_fork(context, vfork);
    }

    return 0;
}

pub fn sys_yield() -> isize {
    unsafe {
        core::arch::asm!("int {}", const InterruptIndex::Timer as u8);
    }

    0
}

pub fn sys_exit(_code: usize) -> isize {
    let task = SCHEDULER.lock().current();

    if let Some(task) = task.upgrade() {
        let mut scheduler = SCHEDULER.lock();
        task.read().process.upgrade().unwrap().read().exit();
        scheduler.remove(Arc::downgrade(&task));
    }

    sys_yield();

    SystemError::ESRCH.to_posix_errno() as isize
}
