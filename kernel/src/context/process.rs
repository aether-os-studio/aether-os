use alloc::collections::btree_map::BTreeMap;
use alloc::ffi::CString;
use alloc::string::String;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt::Debug;
use core::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use error::SystemError;
use object::{File, Object, ObjectSegment};
use spin::{Lazy, RwLock};
use x86_64::VirtAddr;
use x86_64::structures::paging::OffsetPageTable;

use super::context::Context;
use super::thread::{SharedThread, Thread};
use crate::fs::vfs::ROOT;
use crate::fs::vfs::inode::{FileMode, InodeRef};
use crate::memory::{ExtendedPageTable, ref_current_page_table, ref_page_table};
use crate::memory::{FRAME_ALLOCATOR, KERNEL_PAGE_TABLE};
use crate::memory::{MappingType, MemoryManager};

pub(super) type SharedProcess = Arc<RwLock<Process>>;
pub(super) type WeakSharedProcess = Weak<RwLock<Process>>;

pub static KERNEL_PROCESS: Lazy<SharedProcess> = Lazy::new(|| {
    let process = Process::new("kernel", ref_current_page_table());
    let process = Arc::new(RwLock::new(process));
    PROCESSES.write().push(process.clone());
    process
});

static PROCESSES: RwLock<Vec<SharedProcess>> = RwLock::new(Vec::new());

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ProcessId(pub u64);

impl ProcessId {
    fn new() -> Self {
        static NEXT_ID: AtomicU64 = AtomicU64::new(1);
        ProcessId(NEXT_ID.fetch_add(1, Ordering::Relaxed))
    }
}

/// 程序初始化信息，这些信息会被压入用户栈中
#[derive(Debug)]
pub struct ProcInitInfo {
    pub proc_name: CString,
    pub args: Vec<CString>,
    pub envs: Vec<CString>,
    pub auxv: BTreeMap<u8, usize>,
}

pub const BRK_START_BASE_ADDR: usize = 0x700000000000;
pub const DEFAULT_BRK_SIZE: usize = 0x8000;

#[allow(dead_code)]
pub struct Process {
    pub id: ProcessId,
    pub name: String,
    pub page_table: OffsetPageTable<'static>,
    pub threads: Vec<SharedThread>,
    pub init_info: ProcInitInfo,
    pub brk_start: usize,
    pub brk_end: usize,
    pub next_fd: AtomicUsize,
    pub files: BTreeMap<usize, (InodeRef, FileMode, usize)>,
    pub cwd: InodeRef,
}

impl ProcInitInfo {
    pub fn new(proc_name: &str) -> Self {
        Self {
            proc_name: CString::new(proc_name).unwrap_or(CString::new("").unwrap()),
            args: Vec::new(),
            envs: Vec::new(),
            auxv: BTreeMap::new(),
        }
    }
}

impl Process {
    pub fn new(name: &str, page_table: OffsetPageTable<'static>) -> Self {
        Self {
            id: ProcessId::new(),
            name: String::from(name),
            page_table,
            threads: Vec::new(),
            init_info: ProcInitInfo::new(name),
            brk_start: BRK_START_BASE_ADDR,
            brk_end: BRK_START_BASE_ADDR + DEFAULT_BRK_SIZE,
            next_fd: AtomicUsize::new(3),
            files: BTreeMap::new(),
            cwd: ROOT.lock().clone(),
        }
    }

    pub fn exit(&self) {
        let mut processes = PROCESSES.write();
        if let Some(index) = processes
            .iter()
            .position(|process| process.read().id == self.id)
        {
            processes.remove(index);
        }
    }

    pub fn create(name: &str, elf_data: &'static [u8]) {
        let binary = ProcessBinary::parse(elf_data);
        let mut page_table = unsafe { KERNEL_PAGE_TABLE.lock().deep_copy() };
        ProcessBinary::map_segments(&binary, &mut page_table);
        let _ = MemoryManager::alloc_range(
            VirtAddr::new(BRK_START_BASE_ADDR as u64),
            DEFAULT_BRK_SIZE as u64,
            MappingType::UserData.flags(),
            &mut page_table,
        );

        let process = Arc::new(RwLock::new(Self::new(name, page_table)));
        Thread::new_user_thread(Arc::downgrade(&process), binary.entry() as usize);
        PROCESSES.write().push(process.clone());
    }

    pub fn do_fork(&self, context: &mut Context, vfork: bool) -> isize {
        let page_table = if vfork {
            ref_page_table(self.page_table.physical_address())
        } else {
            unsafe { self.page_table.deep_copy() }
        };
        let process = Arc::new(RwLock::new(Self::new(&self.name, page_table)));
        for thread in self.threads.iter() {
            thread.read().do_fork(context, Arc::downgrade(&process));
        }

        PROCESSES.write().push(process.clone());

        process.read().id.0 as isize
    }
}

impl ProcInitInfo {
    /// 把程序初始化信息压入用户栈中
    /// 这个函数会把参数、环境变量、auxv等信息压入用户栈中
    ///
    /// ## 返回值
    ///
    /// 返回值是一个元组，第一个元素是最终的用户栈顶地址，第二个元素是环境变量pointer数组的起始地址     
    pub unsafe fn push_at(
        &self,
        ustack: &mut Context,
        page_table: &mut OffsetPageTable<'static>,
    ) -> Result<(usize, usize), SystemError> {
        // 先把程序的名称压入栈中
        self.push_str(ustack, &self.proc_name, page_table)?;

        // 然后把环境变量压入栈中
        let envps = self
            .envs
            .iter()
            .map(|s| {
                self.push_str(ustack, s, page_table)
                    .expect("push_str failed");
                ustack.rsp
            })
            .collect::<Vec<_>>();
        // 然后把参数压入栈中
        let argps = self
            .args
            .iter()
            .map(|s| {
                self.push_str(ustack, s, page_table)
                    .expect("push_str failed");
                ustack.rsp
            })
            .collect::<Vec<_>>();

        // 压入auxv
        self.push_slice(
            ustack,
            &[core::ptr::null::<u8>(), core::ptr::null::<u8>()],
            page_table,
        )?;
        for (&k, &v) in self.auxv.iter() {
            self.push_slice(ustack, &[k as usize, v], page_table)?;
        }

        // 把环境变量指针压入栈中
        self.push_slice(ustack, &[core::ptr::null::<u8>()], page_table)?;
        self.push_slice(ustack, envps.as_slice(), page_table)?;

        // 把参数指针压入栈中
        self.push_slice(ustack, &[core::ptr::null::<u8>()], page_table)?;
        self.push_slice(ustack, argps.as_slice(), page_table)?;

        let argv_ptr = ustack.rsp;

        // 把argc压入栈中
        self.push_slice(ustack, &[self.args.len()], page_table)?;

        return Ok((ustack.rsp, argv_ptr));
    }

    fn push_slice<T: Copy>(
        &self,
        ustack: &mut Context,
        slice: &[T],
        page_table: &mut OffsetPageTable<'static>,
    ) -> Result<(), SystemError> {
        let mut sp = ustack.rsp;
        sp -= core::mem::size_of_val(slice);
        sp -= sp % core::mem::align_of::<T>();

        page_table.write_to_mapped_address(slice, VirtAddr::new(sp as u64));
        ustack.rsp = sp;

        return Ok(());
    }

    fn push_str(
        &self,
        ustack: &mut Context,
        s: &CString,
        page_table: &mut OffsetPageTable<'static>,
    ) -> Result<(), SystemError> {
        let bytes = s.as_bytes_with_nul();
        self.push_slice(ustack, bytes, page_table)?;
        return Ok(());
    }
}

struct ProcessBinary;

impl ProcessBinary {
    fn parse(bin: &'static [u8]) -> File<'static> {
        File::parse(bin).expect("Failed to parse ELF binary!")
    }

    fn map_segments(elf_file: &File, page_table: &mut OffsetPageTable<'static>) {
        for segment in elf_file.segments() {
            let address = VirtAddr::new(segment.address());

            MemoryManager::alloc_range(
                address,
                segment.size(),
                MappingType::UserCode.flags(),
                page_table,
            )
            .expect("Failed to allocate memory for ELF segment");

            if let Ok(data) = segment.data() {
                page_table.write_to_mapped_address(data, address);
            }
        }
    }
}

impl Drop for Process {
    fn drop(&mut self) {
        unsafe {
            self.page_table.free_user_page_table();
            info!("Process {} dropped", self.id.0);
            info!("Memory usage: {}", FRAME_ALLOCATOR.lock());
        }
    }
}
