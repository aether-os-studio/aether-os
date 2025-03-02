use alloc::collections::{BTreeMap, VecDeque};
use alloc::sync::Weak;
use core::sync::atomic::{AtomicBool, Ordering};
use rmm::VirtualAddress;
use spin::{Lazy, Mutex};

use super::arch::Context;
use super::thread::{Thread, WeakSharedThread};
use crate::arch::smp::CPUS;
use crate::cpu_set::LogicalCpuId;

pub static SCHEDULER_INIT: AtomicBool = AtomicBool::new(false);
pub static SCHEDULER: Lazy<Mutex<Scheduler>> = Lazy::new(|| Mutex::new(Scheduler::default()));

pub fn init() {
    SCHEDULER_INIT.store(true, Ordering::SeqCst);
}

pub struct Scheduler {
    current_threads: BTreeMap<LogicalCpuId, WeakSharedThread>,
    ready_threads: VecDeque<WeakSharedThread>,
}

impl Default for Scheduler {
    fn default() -> Self {
        let current_threads = CPUS
            .read()
            .iter_id()
            .map(|cpu_id| (LogicalCpuId::new(*cpu_id), Thread::get_init_thread()))
            .collect();

        Self {
            current_threads,
            ready_threads: VecDeque::new(),
        }
    }
}

impl Scheduler {
    #[inline]
    pub fn add(&mut self, thread: WeakSharedThread) {
        self.ready_threads.push_back(thread);
    }

    #[inline]
    pub fn remove(&mut self, thread: WeakSharedThread) {
        self.ready_threads
            .retain(|other| !Weak::ptr_eq(other, &thread));
    }

    #[inline]
    pub fn current(&self) -> WeakSharedThread {
        let cpu_id = crate::cpu_id();
        self.current_threads[&cpu_id].clone()
    }
}

impl Scheduler {
    pub fn schedule(&mut self, context: VirtualAddress) -> VirtualAddress {
        let cpu_id = crate::cpu_id();

        if let Some(weak) = self.current_threads.get(&cpu_id) {
            if let Some(thread) = weak.upgrade() {
                let mut thread = thread.write();
                thread.context = Context::from_address(context);

                if !thread.sleeping {
                    self.ready_threads.push_back(weak.clone());
                }
                thread.sleeping = false;
            }
        }

        if let Some(next_thread) = self.ready_threads.pop_front() {
            self.current_threads.insert(cpu_id, next_thread);
        }

        let next_thread = self.current_threads[&cpu_id].upgrade().unwrap();
        let next_thread = next_thread.read();

        let kernel_address = next_thread.kernel_stack.end_address();
        CPUS.write()
            .get_mut(cpu_id.get())
            .set_ring0_rsp(kernel_address);

        next_thread.context.address()
    }
}
