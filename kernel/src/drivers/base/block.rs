use rmm::VirtualAddress;

pub trait BlockDeviceBase: Send + Sync + 'static {
    fn read(&mut self, lba: usize, count: usize, addr: VirtualAddress) -> usize;
    fn write(&mut self, lba: usize, count: usize, addr: VirtualAddress) -> usize;

    fn block_size(&self) -> usize;
    fn block_count(&self) -> usize;
}
