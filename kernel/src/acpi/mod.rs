use core::ptr::NonNull;

use acpi::{madt::Madt, AcpiHandler, AcpiTables, PhysicalMapping};
use alloc::boxed::Box;
use limine::request::RsdpRequest;
use rmm::{Arch, PhysicalAddress};

use crate::{arch::memory::CurrentRmmArch as RmmA, memory::mapper::KernelMapper};

#[used]
#[unsafe(link_section = ".requests")]
static RSDP_REQUEST: RsdpRequest = RsdpRequest::new();

pub fn init() {
    let response = RSDP_REQUEST.get_response().unwrap();
    let rsdp_addr = response.address() as usize;
    let rsdp_addr = rsdp_addr - RmmA::PHYS_OFFSET;

    let acpi_table = {
        let tables = unsafe { AcpiTables::from_rsdp(AcpiMemHandler, rsdp_addr) }
            .expect("Cannot find acpi table");
        log::debug!("Find acpi table successfully");
        Box::leak(Box::new(tables))
    };

    let page_table = &mut KernelMapper::lock();
    unsafe {
        crate::arch::apic::local::init(page_table);
        let madt = acpi_table.find_table::<Madt>().unwrap();
        crate::arch::apic::io::init(page_table, &madt);
    }
}

#[derive(Clone)]
pub struct AcpiMemHandler;

impl AcpiHandler for AcpiMemHandler {
    unsafe fn map_physical_region<T>(
        &self,
        physical_address: usize,
        size: usize,
    ) -> acpi::PhysicalMapping<Self, T> {
        let virtual_address = RmmA::phys_to_virt(PhysicalAddress::new(physical_address)).data();
        let non_virtual = NonNull::new_unchecked(virtual_address as *mut T);
        PhysicalMapping::new(physical_address, non_virtual, size, size, self.clone())
    }

    fn unmap_physical_region<T>(_region: &acpi::PhysicalMapping<Self, T>) {}
}
