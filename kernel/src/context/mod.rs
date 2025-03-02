#[cfg(target_arch = "x86_64")]
#[path = "arch/x86_64.rs"]
mod arch;

pub mod memory;
pub mod process;
pub mod scheduler;
pub mod stack;
pub mod thread;

pub fn init() {
    self::scheduler::init();
}
