use core::{ops::Add, sync::atomic::AtomicBool};

use alloc::collections::{btree_map::BTreeMap, vec_deque::VecDeque};
use rmm::VirtualAddress;
use spin::{Lazy, Mutex, RwLock};

use crate::arch::proc::{arch_get_cpu_id, arch_set_kernel_stack};

use super::{STACK_SIZE, SharedContext};

pub struct Scheduler {
    currents: BTreeMap<u32, SharedContext>,
}

impl Default for Scheduler {
    fn default() -> Self {
        Scheduler {
            currents: BTreeMap::new(),
        }
    }
}

impl Scheduler {
    pub fn schedule(&mut self, prev_context: VirtualAddress) -> VirtualAddress {
        if !SCHEDULER_ENABLED.load(core::sync::atomic::Ordering::SeqCst) {
            return prev_context;
        }

        let cpu_id = arch_get_cpu_id();

        let mut contexts = CONTEXTS.write();

        if let Some(current) = self.currents.get(&cpu_id) {
            current.write().arch.set_address(prev_context);

            contexts.push_back(current.clone());
        }

        if let Some(next) = contexts.pop_front() {
            self.currents.insert(cpu_id, next.clone());
            arch_set_kernel_stack(next.read().kernel_stack.data().add(STACK_SIZE));
            // switch fs/gs base
            next.read().arch.make_current();
            next.read().arch.address()
        } else {
            return prev_context;
        }
    }
}

pub static SCHEDULER_ENABLED: AtomicBool = AtomicBool::new(false);

pub static CONTEXTS: RwLock<VecDeque<SharedContext>> = RwLock::new(VecDeque::new());

unsafe impl Send for Scheduler {}
unsafe impl Sync for Scheduler {}

pub static SCHEDULER: Lazy<Mutex<Scheduler>> = Lazy::new(|| Mutex::new(Scheduler::default()));

pub fn get_current_context() -> SharedContext {
    SCHEDULER
        .lock()
        .currents
        .get(&arch_get_cpu_id())
        .cloned()
        .unwrap()
}

pub fn init() {
    SCHEDULER_ENABLED.store(true, core::sync::atomic::Ordering::SeqCst);
}
