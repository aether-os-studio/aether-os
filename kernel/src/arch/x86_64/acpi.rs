use core::ptr::NonNull;

use acpi::{AcpiHandler, AcpiTables};
use limine::request::RsdpRequest;
use rmm::{Arch, PageFlags, PhysicalAddress, VirtualAddress};
use spin::{Mutex, Once};

use crate::{arch::CurrentMMArch, memory::KERNEL_PAGE_TABLE};

#[used]
#[unsafe(link_section = ".requests")]
static RSDP_REQUEST: RsdpRequest = RsdpRequest::new();

#[derive(Clone)]
pub struct AcpiMemhandler;

impl AcpiHandler for AcpiMemhandler {
    unsafe fn map_physical_region<T>(
        &self,
        physical_address: usize,
        size: usize,
    ) -> acpi::PhysicalMapping<Self, T> {
        let phys: PhysicalAddress = PhysicalAddress::new(physical_address);
        let virt = CurrentMMArch::phys_to_virt(phys);

        for i in 0..((size + CurrentMMArch::PAGE_SIZE - 1) / CurrentMMArch::PAGE_SIZE) {
            let result = KERNEL_PAGE_TABLE.lock().map_phys(
                VirtualAddress::new(virt.data() + i * CurrentMMArch::PAGE_SIZE),
                phys,
                PageFlags::new().write(true),
            );
            if let Some(flusher) = result {
                flusher.flush();
            }
        }

        acpi::PhysicalMapping::new(
            physical_address,
            NonNull::new_unchecked(virt.data() as *mut T),
            size,
            (size + CurrentMMArch::PAGE_SIZE - 1) / CurrentMMArch::PAGE_SIZE
                * CurrentMMArch::PAGE_SIZE,
            self.clone(),
        )
    }

    fn unmap_physical_region<T>(region: &acpi::PhysicalMapping<Self, T>) {
        let size = region.mapped_length();
        let virt = region.virtual_start().as_ptr() as usize;

        for i in 0..((size + CurrentMMArch::PAGE_SIZE - 1) / CurrentMMArch::PAGE_SIZE) {
            unsafe {
                KERNEL_PAGE_TABLE.lock().unmap_phys(
                    VirtualAddress::new(virt + i * CurrentMMArch::PAGE_SIZE),
                    true,
                );
            };
        }
    }
}

pub static ACPI: Once<Mutex<AcpiTables<AcpiMemhandler>>> = Once::new();

pub fn init() {
    let acpi_tables = unsafe {
        AcpiTables::from_rsdp(
            AcpiMemhandler,
            RSDP_REQUEST.get_response().unwrap().address() as usize,
        )
    }
    .unwrap();

    ACPI.call_once(|| Mutex::new(acpi_tables));
}
