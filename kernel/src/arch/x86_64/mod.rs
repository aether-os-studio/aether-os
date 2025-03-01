use ::rmm::Arch;
use limine::request::HhdmRequest;
use memory::CurrentRmmArch as RmmA;

use crate::BASE_REVISION;

pub mod apic;
pub mod ipi;
pub mod memory;
pub mod rmm;
pub mod serial;

pub const STACK_SIZE: usize = 0x100000;

/// Sets the base revision to the latest revision supported by the crate.
/// See specification for further info.
/// Be sure to mark all limine requests with #[used], otherwise they may be removed by the compiler.
#[used]
// The .requests section allows limine to find the requests faster and more safely.
#[unsafe(link_section = ".requests")]
static HHDM_REQUEST: HhdmRequest = HhdmRequest::new();

#[no_mangle]
unsafe extern "C" fn kmain() -> ! {
    assert!(BASE_REVISION.is_supported());
    assert_eq!(
        HHDM_REQUEST.get_response().unwrap().offset() as usize,
        RmmA::PHYS_OFFSET
    );

    crate::klog::init();

    log::info!("Aether OS kernel starting...");

    crate::startup::memory::register_bootloader_areas();
    crate::startup::memory::init(Some(0x100000), None);
    crate::allocator::init();

    crate::acpi::init();

    log::info!("Aether OS initialized");

    crate::hcf();
}
