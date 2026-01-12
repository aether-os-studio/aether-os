use alloc::{
    collections::{btree_map::BTreeMap, vec_deque::VecDeque},
    sync::{Arc, Weak},
};
use spin::{Mutex, RwLock};

use crate::task::{ArcTask, WeakArcTask};

pub type ArcScheduler = Arc<RwLock<Scheduler>>;
pub type WeakArcScheduler = Weak<RwLock<Scheduler>>;

pub struct Scheduler {
    current: Option<WeakArcTask>,
    tasks: VecDeque<WeakArcTask>,
}

impl Scheduler {
    pub fn new() -> ArcScheduler {
        Arc::new(RwLock::new(Scheduler {
            current: None,
            tasks: VecDeque::new(),
        }))
    }

    pub fn get_current_task(&self) -> Option<ArcTask> {
        self.current.as_ref().map(|c| c.upgrade().unwrap())
    }

    pub fn set_current_task(&mut self, current: ArcTask) {
        self.current = Some(Arc::downgrade(&current))
    }

    pub fn add_task(&mut self, task: ArcTask) {
        self.tasks.push_back(Arc::downgrade(&task));
    }

    pub fn schedule(&mut self) -> ArcTask {
        if let Some(current) = self.current.clone() {
            if let Some(next) = self.tasks.pop_front() {
                self.tasks.push_back(current);
                self.current = Some(next.clone());
                next.upgrade().unwrap()
            } else {
                current.upgrade().unwrap()
            }
        } else {
            panic!("Scheduler not initialized");
        }
    }
}

pub static SCHEDULERS: Mutex<BTreeMap<usize, ArcScheduler>> = Mutex::new(BTreeMap::new());
