pub mod ahci;
pub mod block;
pub mod pci;

pub fn init() {
    block::init();
}
