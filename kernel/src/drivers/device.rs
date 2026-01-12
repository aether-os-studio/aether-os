use alloc::{sync::Arc, vec::Vec};
use rmm::PhysicalAddress;
use spin::Mutex;

pub trait MmioDevice: Sync + Send + 'static {
    fn get_mmio_addr_range(&self) -> Option<Vec<(PhysicalAddress, usize)>>;
}

pub static MMIO_DEVICES: Mutex<Vec<Arc<dyn MmioDevice>>> = Mutex::new(Vec::new());

pub trait CharDevice {
    #[allow(dead_code)]
    fn read(&mut self, buf: &mut [u8]);
    fn write(&mut self, buf: &[u8]);
}
