use crate::{
    arch::{CurrentRmmArch, rmm::page_flags},
    drivers::device::{MMIO_DEVICES, MmioDevice},
};
use alloc::{
    string::String,
    sync::{Arc, Weak},
    vec::Vec,
};
use fdt_parser::{Fdt, Node};
use limine::request::DeviceTreeBlobRequest;

#[used]
#[unsafe(link_section = ".requests")]
static DTB_REQUEST: DeviceTreeBlobRequest = DeviceTreeBlobRequest::new();

use rmm::{Arch, PageMapper, PhysicalAddress};
use spin::Mutex;

use crate::init::memory::{FRAME_ALLOCATOR, PAGE_SIZE, align_down, align_up};

pub struct DtbDevice {
    pub(super) node: Node,
}

impl DtbDevice {
    pub fn new(node: Node) -> Self {
        Self { node }
    }
}

impl MmioDevice for DtbDevice {
    fn get_mmio_addr_range(&self) -> Option<Vec<(PhysicalAddress, usize)>> {
        Some(
            self.node
                .reg()
                .ok()?
                .iter()
                .map(|r| {
                    (
                        PhysicalAddress::new(r.address as usize),
                        r.size.unwrap_or(0),
                    )
                })
                .collect::<Vec<_>>(),
        )
    }
}

pub static DTB_DEVICES: Mutex<Vec<Weak<DtbDevice>>> = Mutex::new(Vec::new());

pub unsafe fn init() {
    let dtb_response = DTB_REQUEST.get_response();
    if let Some(dtb_response) = dtb_response {
        let dtb_ptr = dtb_response.dtb_ptr() as *const u8;

        let mut frame_allocator = FRAME_ALLOCATOR.lock();
        let mut mapper = PageMapper::current(rmm::TableKind::Kernel, &mut *frame_allocator);

        let start = align_down(dtb_ptr as usize);
        let end = align_up(start + PAGE_SIZE);

        for addr in (start..end).step_by(PAGE_SIZE) {
            let phys = PhysicalAddress::new(addr);
            let virt = CurrentRmmArch::phys_to_virt(phys);
            let flags = page_flags::<CurrentRmmArch>(virt);
            if let Some(flusher) = mapper.map_phys(virt, phys, flags) {
                flusher.flush();
            }
        }

        let dtb_virt = CurrentRmmArch::phys_to_virt(PhysicalAddress::new(dtb_ptr as usize)).data()
            as *const u32;
        let total_size = u32::from_be(dtb_virt.offset(1).read_volatile());

        let start = align_down(dtb_ptr as usize + PAGE_SIZE);
        let end = align_up(start + total_size as usize);

        for addr in (start..end).step_by(PAGE_SIZE) {
            let phys = PhysicalAddress::new(addr);
            let virt = CurrentRmmArch::phys_to_virt(phys);
            let flags = page_flags::<CurrentRmmArch>(virt);
            if let Some(flusher) = mapper.map_phys(virt, phys, flags) {
                flusher.flush();
            }
        }

        let fdt = Fdt::from_ptr(dtb_ptr as *mut u8).unwrap();
        for node in fdt.all_nodes() {
            let device = Arc::new(DtbDevice::new(node));
            MMIO_DEVICES.lock().push(device.clone());
            DTB_DEVICES.lock().push(Arc::downgrade(&device));
        }
    }
}

pub fn iter_device_by_compatible(compatible: String, callback: fn(device: Arc<DtbDevice>)) {
    for device in DTB_DEVICES.lock().iter() {
        let device = device.upgrade().unwrap();
        if device.node.compatibles().contains(&compatible) {
            callback(device.clone())
        }
    }
}
