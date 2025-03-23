use core::panic::PanicInfo;
use core::slice::from_raw_parts;
use limine::request::ExecutableFileRequest;
use object::{File, Object, ObjectSymbol};
use rustc_demangle::demangle;
use spin::Lazy;
use x86_64::{VirtAddr, structures::paging::Translate};

use crate::{memory::KERNEL_PAGE_TABLE, pctable::idt::StackTrace};

type ExeRequest = ExecutableFileRequest;

#[used]
#[unsafe(link_section = ".requests")]
static EXE_REQUEST: ExeRequest = ExeRequest::new();

static KERNEL_FILE: Lazy<File> = Lazy::new(|| unsafe {
    let kernel = EXE_REQUEST.get_response().unwrap().file();
    let bin = from_raw_parts(kernel.addr(), kernel.size() as _);
    File::parse(bin).expect("Failed to parse kernel file")
});

#[inline(never)]
pub unsafe fn stack_trace() {
    let mut frames = StackTrace::start();

    error!("RUST PANIC BACKTRACE:");

    //Maximum 64 frames
    for _ in 0..64 {
        if let Some(frame) = frames {
            let fp_virt = VirtAddr::new(frame.fp as u64);
            let pc_virt = VirtAddr::new(frame.pc_ptr as u64);

            if fp_virt == VirtAddr::zero() || pc_virt == VirtAddr::zero() {
                break;
            }

            if KERNEL_PAGE_TABLE.lock().translate_addr(fp_virt).is_some()
                && KERNEL_PAGE_TABLE.lock().translate_addr(pc_virt).is_some()
            {
                let pc = *frame.pc_ptr;

                if pc == 0 {
                    info!(" {:#018X}: EMPTY RETURN", frame.fp);
                    break;
                } else {
                    info!("  FP {:#018X}: PC {:#018X}", frame.fp, pc);
                    symbol_trace(pc);
                    frames = frame.next();
                }
            } else {
                info!("  {:#018X}: GUARD PAGE", frame.fp);
                break;
            }
        } else {
            break;
        }
    }
}

#[inline(never)]
pub unsafe fn symbol_trace(addr: usize) {
    if let Some(symbol) = KERNEL_FILE.symbols().find(|symbol| {
        let start = symbol.address();
        let end = start + symbol.size();
        (start..end).contains(&(addr as u64))
    }) {
        info!(
            "    {:#018X}+{:#04X}",
            symbol.address(),
            addr - symbol.address() as usize
        );

        if let Ok(name) = symbol.name() {
            info!("    {:#}", demangle(name));
        }
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("{}", info);

    unsafe { stack_trace() };

    crate::hcf()
}
