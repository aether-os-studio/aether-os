use crate::BASE_REVISION;

pub mod memory;
pub mod rmm;
pub mod serial;

pub const STACK_SIZE: usize = 0x100000;

#[no_mangle]
unsafe extern "C" fn kmain() -> ! {
    assert!(BASE_REVISION.is_supported());

    crate::klog::init();

    log::info!("Aether OS kernel starting...");

    crate::startup::memory::register_bootloader_areas();
    crate::startup::memory::init(Some(0x100000), None);
    memory::paging::init();
    crate::allocator::init();

    log::info!("Aether OS initialized");

    crate::hcf();
}
