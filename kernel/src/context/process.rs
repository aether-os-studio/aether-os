use alloc::string::String;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt::Debug;
use core::sync::atomic::{AtomicU64, Ordering};
use spin::{Lazy, RwLock};

use super::memory::AddrSpaceWrapper;
use super::thread::SharedThread;

pub(super) type SharedProcess = Arc<RwLock<Process>>;
pub(super) type WeakSharedProcess = Weak<RwLock<Process>>;

pub static KERNEL_PROCESS: Lazy<SharedProcess> = Lazy::new(|| {
    let process = Process::new(
        "kernel",
        AddrSpaceWrapper::new().expect("Cannot create kernel process page table"),
    );
    let process = Arc::new(RwLock::new(process));
    PROCESSES.write().push(process.clone());
    process
});

static PROCESSES: RwLock<Vec<SharedProcess>> = RwLock::new(Vec::new());

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ProcessId(pub u64);

impl ProcessId {
    fn new() -> Self {
        static NEXT_ID: AtomicU64 = AtomicU64::new(0);
        ProcessId(NEXT_ID.fetch_add(1, Ordering::Relaxed))
    }
}

#[allow(dead_code)]
pub struct Process {
    pub id: ProcessId,
    pub name: String,
    pub page_table: Arc<AddrSpaceWrapper>,
    pub threads: Vec<SharedThread>,
}

impl Process {
    pub fn new(name: &str, page_table: Arc<AddrSpaceWrapper>) -> Self {
        Self {
            id: ProcessId::new(),
            name: String::from(name),
            page_table,
            threads: Vec::new(),
        }
    }

    pub fn exit(&self) {
        let mut processes = PROCESSES.write();
        if let Some(index) = processes
            .iter()
            .position(|process| process.read().id == self.id)
        {
            processes.remove(index);
        }
    }
}
