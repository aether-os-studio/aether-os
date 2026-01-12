mod boot;
pub mod cache;
pub mod drivers;
pub mod irq;
pub mod rmm;
pub mod smp;
pub mod time;

pub use self::cache::AArch64CacheArch as CurrentCacheArch;
pub use self::irq::AArch64IrqArch as CurrentIrqArch;
pub use self::time::AArch64TimeArch as CurrentTimeArch;
pub use ::rmm::AArch64Arch as CurrentRmmArch;

pub fn early_init() {
    rmm::init();
    irq::init();
    smp::init();
    drivers::gic::init();
    drivers::timer::init();
}
