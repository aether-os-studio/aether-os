use core::alloc::Layout;

use alloc::{collections::btree_map::BTreeMap, sync::Arc, vec::Vec};
use nvme::{Allocator, Device, IoQueuePair, Namespace};
use pci_types::device_type::DeviceType;
use rmm::{Arch, PageFlags, PhysicalAddress, VirtualAddress};
use spin::{Lazy, Mutex};

use crate::{
    arch::CurrentMMArch,
    drivers::{base::block::BlockDeviceBase, pci::PCI_DEVICES},
    memory::KERNEL_PAGE_TABLE,
};

pub static ALLOCATED_MEMORYS: Mutex<BTreeMap<usize, Layout>> = Mutex::new(BTreeMap::new());

pub struct NvmeAllocator;

impl Allocator for NvmeAllocator {
    unsafe fn allocate(&self, size: usize) -> usize {
        let layout = Layout::from_size_align(size, CurrentMMArch::PAGE_SIZE).unwrap();
        let addr = alloc::alloc::alloc(layout.clone()) as usize;

        ALLOCATED_MEMORYS.lock().insert(addr, layout);

        addr
    }

    unsafe fn deallocate(&self, addr: usize) {
        let allocated_memorys = ALLOCATED_MEMORYS.lock();
        let layout = allocated_memorys.get(&addr).unwrap();

        alloc::alloc::dealloc(addr as *mut u8, layout.clone());
    }

    fn translate(&self, addr: usize) -> usize {
        KERNEL_PAGE_TABLE
            .lock()
            .translate(VirtualAddress::new(addr))
            .unwrap()
            .0
            .data()
            + (addr & CurrentMMArch::PAGE_OFFSET_MASK)
    }
}

type SharedNvmeDevice = Arc<Mutex<Device<NvmeAllocator>>>;
type LockedQueuePair = Mutex<IoQueuePair<NvmeAllocator>>;

pub struct NvmeBlockDevice {
    pub namespace: Namespace,
    pub qpairs: LockedQueuePair,
}

pub struct NvmeManager(Vec<SharedNvmeDevice>);

impl NvmeManager {
    pub fn iter(&self) -> impl Iterator<Item = Vec<NvmeBlockDevice>> {
        self.0.iter().map(|device| {
            let mut controller = device.lock();
            let namespaces = controller.identify_namespaces(0).unwrap();

            let mapper = |namespace: Namespace| {
                let qpair = controller
                    .create_io_queue_pair(namespace.clone(), 64)
                    .ok()?;

                Some(NvmeBlockDevice {
                    namespace,
                    qpairs: Mutex::new(qpair),
                })
            };

            namespaces.into_iter().filter_map(mapper).collect()
        })
    }
}

pub static NVME: Lazy<NvmeManager> = Lazy::new(|| {
    let mut connections = Vec::new();

    for device in PCI_DEVICES.lock().iter() {
        if device.device_type == DeviceType::NvmeController {
            if let Some(bar0) = device.bars[0] {
                let (addr, size) = bar0.unwrap_mem();
                let phys = PhysicalAddress::new(addr);
                let virt = unsafe { CurrentMMArch::phys_to_virt(phys) };

                unsafe {
                    for i in 0..(size + CurrentMMArch::PAGE_SIZE - 1) / CurrentMMArch::PAGE_SIZE {
                        let result = KERNEL_PAGE_TABLE.lock().map_phys(
                            virt.add(i * CurrentMMArch::PAGE_SIZE),
                            phys.add(i * CurrentMMArch::PAGE_SIZE),
                            PageFlags::new().write(true),
                        );

                        if let Some(flusher) = result {
                            flusher.flush();
                        }
                    }
                }

                let virtual_address = virt.data();
                let device = Device::init(virtual_address, NvmeAllocator).unwrap();
                connections.push(Arc::new(Mutex::new(device)));
            }
        }
    }

    NvmeManager(connections)
});

impl BlockDeviceBase for NvmeBlockDevice {
    fn read(&mut self, lba: usize, count: usize, addr: VirtualAddress) -> usize {
        let bytes = self.namespace.block_size() as usize * count;
        self.qpairs
            .lock()
            .read(addr.data() as *mut u8, bytes, lba as u64)
            .expect("Failed write NVMe");

        return bytes;
    }

    fn write(&mut self, lba: usize, count: usize, addr: VirtualAddress) -> usize {
        let bytes = self.namespace.block_size() as usize * count;
        self.qpairs
            .lock()
            .write(addr.data() as *mut u8, bytes, lba as u64)
            .expect("Failed write NVMe");

        return bytes;
    }

    fn block_size(&self) -> usize {
        self.namespace.block_size() as usize
    }

    fn block_count(&self) -> usize {
        self.namespace.block_count() as usize
    }
}
