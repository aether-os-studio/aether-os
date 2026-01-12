use alloc::collections::btree_map::BTreeMap;
use limine::request::MpRequest;
use spin::Mutex;

#[used]
#[unsafe(link_section = ".requests")]
pub static MP_REQUEST: MpRequest = MpRequest::new();

use core::sync::atomic::AtomicUsize;

pub static BSP_CPUARCHID: AtomicUsize = AtomicUsize::new(0);
pub static CPU_COUNT: AtomicUsize = AtomicUsize::new(0);

pub static CPUID_TO_ARCHID: Mutex<BTreeMap<usize, usize>> = Mutex::new(BTreeMap::new());

pub fn get_cpuid_by_archid(archid_match: usize) -> usize {
    for (&cpuid, &archid) in CPUID_TO_ARCHID.lock().iter() {
        if archid == archid_match {
            return cpuid;
        }
    }
    panic!("Failed to get cpuid by archid");
}

pub fn get_archid_by_cpuid(cpuid_match: usize) -> usize {
    for (&cpuid, &archid) in CPUID_TO_ARCHID.lock().iter() {
        if cpuid == cpuid_match {
            return archid;
        }
    }
    panic!("Failed to get archid by cpuid");
}

pub fn init() {
    crate::arch::smp::init();
}
