use core::sync::atomic::{AtomicBool, Ordering};

use alloc::collections::{btree_map::BTreeMap, vec_deque::VecDeque};
use rmm::VirtualAddress;
use spin::{Lazy, Mutex, RwLock};

use crate::{
    CPU_COUNT,
    arch::{
        mp::CPUS,
        proc::{arch_get_cpu_id, arch_set_kernel_stack},
    },
    proc::STACK_SIZE,
};

use super::SharedContext;

pub struct Scheduler {
    pub currents: BTreeMap<u32, SharedContext>,
    idx: usize,
}

impl Default for Scheduler {
    fn default() -> Self {
        Scheduler {
            currents: BTreeMap::new(),
            idx: 0,
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

        let queue = contexts.get_mut(&cpu_id).unwrap();

        if let Some(new) = queue.pop_front() {
            if let Some(current) = self.currents.get(&cpu_id) {
                current.write().arch.set_address(prev_context);
                queue.push_back(current.clone());
            }

            self.currents.insert(cpu_id, new.clone());

            arch_set_kernel_stack(new.read().kernel_stack.add(STACK_SIZE).data());

            new.read().arch.make_current();
            new.read().arch.address()
        } else {
            prev_context
        }
    }

    pub fn add(&mut self, context: SharedContext) {
        let mut contexts = CONTEXTS.write();

        let cpus = CPUS.write();
        let mut cpus_iter = cpus.iter_id();
        let cpu_id = cpus_iter.nth(self.idx).unwrap();
        self.idx = (self.idx + 1) % CPU_COUNT.load(Ordering::SeqCst);

        let queue = contexts.get_mut(&cpu_id).unwrap();

        queue.push_back(context);
    }
}

pub static SCHEDULER_ENABLED: AtomicBool = AtomicBool::new(false);

pub static CONTEXTS: RwLock<BTreeMap<u32, VecDeque<SharedContext>>> = RwLock::new(BTreeMap::new());

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
    CPUS.read().iter_id().for_each(|&id| {
        CONTEXTS.write().insert(id, VecDeque::new());
    });

    SCHEDULER_ENABLED.store(true, Ordering::SeqCst);
}
