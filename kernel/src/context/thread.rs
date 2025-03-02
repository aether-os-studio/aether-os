use alloc::sync::{Arc, Weak};
use core::fmt::Debug;
use core::sync::atomic::{AtomicU64, Ordering};
use spin::RwLock;

use crate::arch::gdt::Selectors;

use super::process::{WeakSharedProcess, KERNEL_PROCESS};
use super::scheduler::SCHEDULER;
use super::stack::KernelStack;

pub(super) type SharedThread = Arc<RwLock<Thread>>;
pub(super) type WeakSharedThread = Weak<RwLock<Thread>>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ThreadId(pub u64);

impl ThreadId {
    fn new() -> Self {
        static NEXT_ID: AtomicU64 = AtomicU64::new(0);
        ThreadId(NEXT_ID.fetch_add(1, Ordering::Relaxed))
    }
}

pub struct Thread {
    pub id: ThreadId,
    pub kernel_stack: KernelStack,
    pub context: super::arch::Context,
    pub process: WeakSharedProcess,
    pub sleeping: bool,
}

impl Thread {
    pub fn new(process: WeakSharedProcess) -> Self {
        Self {
            id: ThreadId::new(),
            context: super::arch::Context::default(),
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

    pub fn new_kernel_thread(function: fn() -> !) {
        let mut thread = Self::new(Arc::downgrade(&KERNEL_PROCESS));

        thread.context.init(
            function as usize,
            thread.kernel_stack.end_address(),
            KERNEL_PROCESS.read().page_table.get_phys(),
            Selectors::get_kernel_segments(),
        );

        let thread = Arc::new(RwLock::new(thread));
        KERNEL_PROCESS.write().threads.push(thread.clone());

        SCHEDULER.lock().add(Arc::downgrade(&thread));
    }
}
