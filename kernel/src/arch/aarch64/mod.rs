mod boot;
pub mod cache;
pub mod drivers;
pub mod irq;
pub mod rmm;
pub mod smp;
pub mod time;

use crate::task::ArcTask;

pub use self::cache::AArch64CacheArch as CurrentCacheArch;
pub use self::irq::AArch64IrqArch as CurrentIrqArch;
pub use self::irq::Ptrace;
pub use self::smp::get_mpidr as get_archid;
pub use self::time::AArch64TimeArch as CurrentTimeArch;
pub use ::rmm::AArch64Arch as CurrentRmmArch;

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ArchContext {
    pub ip: usize,
    pub sp: usize,
}

pub fn switch_to(_prev: ArcTask, _next: ArcTask) {}

pub fn early_init() {
    rmm::init();
    irq::init();
    smp::init();
    drivers::gic::init();
    drivers::timer::init();
}
