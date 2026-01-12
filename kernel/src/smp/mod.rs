use limine::request::MpRequest;

#[used]
#[unsafe(link_section = ".requests")]
pub static MP_REQUEST: MpRequest = MpRequest::new();

use core::sync::atomic::AtomicUsize;

pub static BSP_CPUARCHID: AtomicUsize = AtomicUsize::new(0);
pub static CPU_COUNT: AtomicUsize = AtomicUsize::new(0);

pub fn init() {
    crate::arch::smp::init();
}
