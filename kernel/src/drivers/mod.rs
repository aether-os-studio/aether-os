pub mod base;
pub mod block;
pub mod display;
pub mod logger;
pub mod pci;
#[cfg(target_arch = "x86_64")]
pub mod ps2;
pub mod term;

pub fn init() {
    logger::init();
    pci::init();
    block::init();
}

pub fn init_after_proc() {
    #[cfg(target_arch = "x86_64")]
    ps2::init();
}
