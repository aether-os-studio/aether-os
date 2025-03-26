use alloc::{sync::Arc, vec::Vec};
use spin::Mutex;

use super::ahci::AHCI;

pub trait BlockDevice: Send + Sync + 'static {
    fn read_block(&self, start_sec: usize, buf: &mut [u8]) -> Option<()>;
    fn write_block(&self, start_sec: usize, buf: &[u8]) -> Option<()>;

    fn get_size(&self) -> usize;
}

struct AHCIDisk {
    num: usize,
}

impl BlockDevice for AHCIDisk {
    fn read_block(&self, start_sec: usize, buf: &mut [u8]) -> Option<()> {
        AHCI.lock()
            .get_mut(self.num)
            .unwrap()
            .read_block(start_sec as u64, buf);

        Some(())
    }

    fn write_block(&self, start_sec: usize, buf: &[u8]) -> Option<()> {
        AHCI.lock()
            .get_mut(self.num)
            .unwrap()
            .write_block(start_sec as u64, buf);

        Some(())
    }

    fn get_size(&self) -> usize {
        AHCI.lock()
            .get_mut(self.num)
            .unwrap()
            .identity()
            .lba48_sectors as usize
            * 512
    }
}

pub static HD_LIST: Mutex<Vec<Arc<dyn BlockDevice>>> = Mutex::new(Vec::new());

pub fn init() {
    let ahci_disk_num = AHCI.lock().len();

    for num in 0..ahci_disk_num {
        let disk = Arc::new(AHCIDisk { num });
        HD_LIST.lock().push(disk.clone());
    }
}
