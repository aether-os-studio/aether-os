pub mod base;
pub mod block;
pub mod display;
pub mod logger;
pub mod pci;
pub mod term;

pub fn init() {
    logger::init();
    pci::init();
    block::init();
}
