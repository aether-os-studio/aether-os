use ::rmm::Arch;
use limine::request::HhdmRequest;
use memory::CurrentRmmArch as RmmA;
use smp::{BSP_LAPIC_ID, CPUS};

use crate::BASE_REVISION;

pub mod apic;
pub mod gdt;
pub mod hpet;
pub mod idt;
pub mod ipi;
pub mod memory;
pub mod rmm;
pub mod serial;
pub mod smp;

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

    CPUS.write().load(*BSP_LAPIC_ID);
    CPUS.write().init_ap();

    crate::acpi::init();
    idt::init();

    crate::context::init();
    crate::context::thread::Thread::new_kernel_thread(crate::start_kernel);

    log::info!("Aether OS initialized");

    x86_64::instructions::interrupts::enable();
    crate::hcf();
}
