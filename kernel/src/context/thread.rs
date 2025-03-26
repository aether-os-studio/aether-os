use alloc::sync::{Arc, Weak};
use core::fmt::Debug;
use core::sync::atomic::{AtomicU64, Ordering};
use spin::RwLock;

use super::context::Context;
use super::process::{KERNEL_PROCESS, WeakSharedProcess};
use super::scheduler::SCHEDULER;
use super::stack::{KernelStack, UserStack};
use crate::memory::{ExtendedPageTable, KERNEL_PAGE_TABLE};
use crate::pctable::gdt::Selectors;

pub(super) type SharedThread = Arc<RwLock<Thread>>;
pub(super) type WeakSharedThread = Weak<RwLock<Thread>>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ThreadId(pub u64);

impl ThreadId {
    fn new() -> Self {
        static NEXT_ID: AtomicU64 = AtomicU64::new(1);
        ThreadId(NEXT_ID.fetch_add(1, Ordering::Relaxed))
    }
}

pub struct Thread {
    pub id: ThreadId,
    pub kernel_stack: KernelStack,
    pub context: Context,
    pub process: WeakSharedProcess,
    pub sleeping: bool,
}

impl Thread {
    pub fn new(process: WeakSharedProcess) -> Self {
        Self {
            id: ThreadId::new(),
            context: Context::default(),
            kernel_stack: KernelStack::default(),
            process,
            sleeping: false,
        }
    }

    pub fn get_init_thread() -> WeakSharedThread {
        let thread = Self::new(Arc::downgrade(&KERNEL_PROCESS));
        let thread = Arc::new(RwLock::new(thread));
        KERNEL_PROCESS.write().threads.push(thread.clone());
        Arc::downgrade(&thread)
    }

    pub fn new_kernel_thread(function: fn()) {
        let mut thread = Self::new(Arc::downgrade(&KERNEL_PROCESS));

        thread.context.init(
            function as usize,
            thread.kernel_stack.end_address(),
            KERNEL_PAGE_TABLE.lock().physical_address(),
            Selectors::get_kernel_segments(),
        );

        let thread = Arc::new(RwLock::new(thread));
        KERNEL_PROCESS.write().threads.push(thread.clone());

        SCHEDULER.lock().add(Arc::downgrade(&thread));
    }

    pub fn new_user_thread(process: WeakSharedProcess, entry_point: usize) {
        let mut thread = Self::new(process.clone());
        let process = process.upgrade().unwrap();
        let mut process = process.write();
        UserStack::map(&mut process.page_table);

        thread.context.init(
            entry_point,
            UserStack::end_address(),
            process.page_table.physical_address(),
            Selectors::get_user_segments(),
        );

        let thread = Arc::new(RwLock::new(thread));
        process.threads.push(thread.clone());

        SCHEDULER.lock().add(Arc::downgrade(&thread));
    }

    pub fn do_fork(&self, context: &mut Context, process: WeakSharedProcess) -> SharedThread {
        let mut thread = Self::new(process.clone());
        let process = process.upgrade().unwrap();
        let mut process = process.write();

        thread.context.cr3 = process.page_table.physical_address().as_u64() as usize;

        thread.context.ss = context.ss;
        thread.context.rsp = context.rsp;
        thread.context.rflags = context.rflags;
        thread.context.cs = context.cs;
        thread.context.rip = context.rip;

        thread.context.rdi = context.rdi;
        thread.context.rsi = context.rsi;

        thread.context.rax = 0;
        thread.context.rbx = context.rbx;
        thread.context.rdx = context.rdx;

        thread.context.r9 = context.r9;
        thread.context.r10 = context.r10;
        thread.context.r12 = context.r12;
        thread.context.r13 = context.r13;
        thread.context.r14 = context.r14;
        thread.context.r15 = context.r15;

        let thread = Arc::new(RwLock::new(thread));
        process.threads.push(thread.clone());

        SCHEDULER.lock().add(Arc::downgrade(&thread));
        thread
    }
}
