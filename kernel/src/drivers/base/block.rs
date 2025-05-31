use rmm::VirtualAddress;

pub trait BlockDeviceBase {
    fn read(&mut self, id: usize, lba: usize, count: usize, addr: VirtualAddress) -> usize;
    fn write(&mut self, id: usize, lba: usize, count: usize, addr: VirtualAddress) -> usize;
    fn block_size(&self) -> usize;
    fn block_count(&self) -> usize;
}
