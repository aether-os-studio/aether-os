pub mod block;
pub mod cache;
pub mod nvme;
pub mod partition;

pub fn init() {
    block::init();
    partition::init().unwrap();
}
