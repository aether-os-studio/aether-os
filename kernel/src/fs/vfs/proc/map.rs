use alloc::{string::String, sync::Arc, vec::Vec};
use spin::RwLock;
use x86_64::{
    VirtAddr,
    structures::paging::{PageSize, PageTableFlags, Size4KiB, Translate, mapper::TranslateResult},
};

use crate::{
    context::process::Process,
    fs::vfs::inode::{Inode, InodeRef},
};

pub struct MemoryMapResult {
    addr: VirtAddr,
    size: usize,
    flags: PageTableFlags,
}

pub struct ProcMapFS {
    path: String,
    process: Arc<RwLock<Process>>,
}

impl ProcMapFS {
    pub fn new(process: Arc<RwLock<Process>>) -> InodeRef {
        Arc::new(RwLock::new(ProcMapFS {
            path: String::new(),
            process,
        }))
    }
}

impl Inode for ProcMapFS {
    fn when_mounted(&mut self, path: String, _father: Option<InodeRef>) {
        self.path.clear();
        self.path.push_str(path.as_str());
    }

    fn when_umounted(&mut self) {}

    fn get_path(&self) -> String {
        self.path.clone()
    }

    #[allow(unused_variables)]
    fn read_at(&self, offset: usize, buf: &mut [u8]) -> Result<usize, error::SystemError> {
        let current_process = self.process.clone();

        let page_table = &current_process.read().page_table;

        let load_start = current_process.read().load_start;
        let load_end = current_process.read().load_end;

        let load_start_pfn = load_start / Size4KiB::SIZE as usize;
        let load_end_pfn = load_end / Size4KiB::SIZE as usize;

        let mut memory_map_result = Vec::new();

        for idx in load_start_pfn..load_end_pfn {
            let vaddr = idx * Size4KiB::SIZE as usize;

            let translate_result = page_table.translate(VirtAddr::new(vaddr as u64));

            if let TranslateResult::Mapped {
                frame,
                offset,
                flags,
            } = translate_result
            {
                memory_map_result.push(MemoryMapResult {
                    addr: VirtAddr::new(vaddr as u64),
                    size: Size4KiB::SIZE as usize,
                    flags,
                });
            } else {
                continue;
            }
        }

        let mut return_result = String::new();

        for result in memory_map_result.iter() {
            let mut line_string = String::new();

            line_string.push_str(format!("{:#x}", result.addr.as_u64()).as_str());
            line_string.push_str("-");
            line_string
                .push_str(format!("{:#x}", result.addr.as_u64() + result.size as u64).as_str());

            line_string.push_str(" ");

            let mut attr_buf = [b'r', b'-', b'-', b'-'];
            if result.flags.contains(PageTableFlags::WRITABLE) {
                attr_buf[1] = b'w';
            }
            if !result.flags.contains(PageTableFlags::NO_EXECUTE) {
                attr_buf[2] = b'x';
            }
            if result.flags.contains(PageTableFlags::PRESENT) {
                attr_buf[3] = b'p';
            }

            let attr_string = str::from_utf8(&attr_buf).unwrap();

            line_string.push_str(attr_string);

            line_string.push_str(" ");

            line_string.push_str(format!("{:#x}", result.size).as_str());

            line_string.push_str(" ");

            line_string.push_str("00:00");

            line_string.push_str(" ");

            line_string.push_str(format!("{}", current_process.read().id.0).as_str());

            line_string.push_str("\t");

            line_string.push_str(current_process.read().name.as_str());

            line_string.push('\n');

            return_result.push_str(line_string.as_str());
        }

        return_result.push('\0');

        let target_buf_size = buf.len();

        buf[..target_buf_size]
            .copy_from_slice(&return_result.as_bytes()[offset..(offset + target_buf_size)]);

        return Ok(target_buf_size);
    }
}
