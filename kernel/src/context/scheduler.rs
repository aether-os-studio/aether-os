use alloc::collections::{BTreeMap, VecDeque};
use alloc::sync::Weak;
use core::sync::atomic::{AtomicBool, Ordering};
use spin::{Lazy, Mutex, RwLock};
use x86_64::VirtAddr;

use crate::apic::LAPIC;
use crate::smp::CPUS;

use super::ctx::Context;
use super::task::Task;

pub static SCHEDULER_INIT: AtomicBool = AtomicBool::new(false);
pub static SCHEDULER: Lazy<Mutex<Scheduler>> = Lazy::new(|| Mutex::new(Scheduler::default()));

pub fn init() {
    x86_64::instructions::interrupts::enable();
    SCHEDULER_INIT.store(true, Ordering::SeqCst);
    log::info!("Scheduler initialized, interrupts enabled!");
}

pub struct Scheduler {
    current_threads: BTreeMap<u32, Weak<RwLock<Task>>>,
    ready_threads: VecDeque<Weak<RwLock<Task>>>,
}

impl Default for Scheduler {
    fn default() -> Self {
        Self {
            current_threads: BTreeMap::new(),
            ready_threads: VecDeque::new(),
        }
    }
}

impl Scheduler {
    #[inline]
    pub fn add(&mut self, thread: Weak<RwLock<Task>>) {
        self.ready_threads.push_back(thread);
    }

    #[inline]
    pub fn remove(&mut self, thread: Weak<RwLock<Task>>) {
        self.ready_threads
            .retain(|other| !Weak::ptr_eq(other, &thread));
    }

    #[inline]
    pub fn current(&self) -> Weak<RwLock<Task>> {
        let lapic_id = unsafe { LAPIC.lock().id() };
        self.current_threads[&lapic_id].clone()
    }
}

impl Scheduler {
    pub fn schedule(&mut self, context: VirtAddr) -> VirtAddr {
        let lapic_id = unsafe { LAPIC.lock().id() };

        if let Some(weak) = self.current_threads.get(&lapic_id) {
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
            self.current_threads.insert(lapic_id, next_thread);
        }

        if let Some(next_thread) = self.current_threads.get(&lapic_id) {
            let next_thread = next_thread.upgrade().unwrap();
            let next_thread = next_thread.read();

            let kernel_address = next_thread.kernel_stack.stack_top();
            CPUS.write().get_mut(lapic_id).set_ring0_rsp(kernel_address);

            return next_thread.context.address();
        }

        context
    }
}
