use alloc::{
    collections::{btree_map::BTreeMap, vec_deque::VecDeque},
    sync::Arc,
    vec::Vec,
};
use rmm::VirtualAddress;
use spin::{Lazy, Mutex, RwLock};

use crate::arch::proc::{arch_get_cpu_id, arch_set_kernel_stack};

use super::{SharedContext, WeakSharedContext};

pub struct Scheduler {
    currents: BTreeMap<u32, WeakSharedContext>,
    contexts: VecDeque<WeakSharedContext>,
}

impl Default for Scheduler {
    fn default() -> Self {
        Scheduler {
            currents: BTreeMap::new(),
            contexts: VecDeque::new(),
        }
    }
}

impl Scheduler {
    pub fn schedule(&mut self, prev_context: VirtualAddress) -> VirtualAddress {
        let cpu_id = arch_get_cpu_id();

        if let Some(current) = self.currents.get(&cpu_id) {
            current
                .upgrade()
                .unwrap()
                .write()
                .arch
                .set_address(prev_context);
        }

        if let Some(next) = self.contexts.pop_front() {
            self.currents.insert(cpu_id, next.clone());
            arch_set_kernel_stack(next.upgrade().unwrap().read().kernel_stack.data());
            next.upgrade().unwrap().read().arch.address()
        } else {
            return prev_context;
        }
    }
}

pub static CONTEXTS: RwLock<VecDeque<SharedContext>> = RwLock::new(VecDeque::new());

impl Scheduler {
    pub fn add(&mut self, context: SharedContext) {
        CONTEXTS.write().push_back(context.clone());
        self.contexts.push_back(Arc::downgrade(&context));
    }
}

unsafe impl Send for Scheduler {}
unsafe impl Sync for Scheduler {}

pub static SCHEDULER: Lazy<Mutex<Scheduler>> = Lazy::new(|| Mutex::new(Scheduler::default()));
