pub mod base;
pub mod block;
pub mod pci;

pub fn init() {
    pci::init();
    block::init();
}
