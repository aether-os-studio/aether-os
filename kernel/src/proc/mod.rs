use core::sync::atomic::{AtomicUsize, Ordering};

use alloc::collections::btree_map::BTreeMap;
use alloc::string::ToString;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use alloc::{boxed::Box, string::String};
use exec::ProcInitInfo;
use rmm::{Arch, FrameAllocator, FrameCount, PageFlags, PhysicalAddress, VirtualAddress};
use sched::SCHEDULER;
use spin::RwLock;

use crate::arch::CurrentMMArch;
use crate::arch::nr::{SigAction, SignalStack};
use crate::arch::rmm::PageMapper;
use crate::fs::StdioIndexNode;
use crate::fs::fd::init_file_descriptor_manager_with_stdin_stdout;
use crate::memory::frame::TheFrameAllocator;
use crate::syscall::Result;
use crate::{arch::proc::context::ContextRegs, memory::FRAME_ALLOCATOR};

pub mod abi;
pub mod exec;
pub mod sched;
pub mod signal;

pub trait ArchContext {
    /// 内核线程初始化
    fn init(&mut self, entry: usize, stack: usize);
    /// 前往用户态
    fn go_to_user(&mut self, entry: usize, stack: usize);
    /// 复制自己
    fn clone(&self) -> Box<dyn ArchContext>;
    /// 获取地址
    fn address(&self) -> VirtualAddress;
    /// 设置寄存器地址
    fn set_address(&mut self, addr: VirtualAddress);
    /// 获取页表地址
    fn page_table_address(&self) -> PhysicalAddress;
    /// 切换到当前
    fn make_current(&self);
    /// 设置架构信息
    #[cfg(target_arch = "x86_64")]
    fn get_fs(&self) -> usize;
    #[cfg(target_arch = "x86_64")]
    fn set_fs(&mut self, fsbase: usize);
}

pub const STACK_SIZE: usize = 32768;

/// 最小调度单位
pub struct Context {
    pid: usize,
    arch: Box<dyn ArchContext>,
    name: String,
    kernel_stack: VirtualAddress,
    init_info: Option<ProcInitInfo>,
    brk_start: usize,
    brk_end: usize,
    actions: BTreeMap<usize, SigAction>,
    signal: u64,
    blocked: u64,
    signal_stack: Option<SignalStack>,
    pub mapped_regions: Vec<(usize, usize)>,
}

pub const BRK_START: usize = 0x0000_7000_0000_0000;

unsafe impl Sync for Context {}
unsafe impl Send for Context {}

static NEXT_PID: AtomicUsize = AtomicUsize::new(0);

impl Context {
    pub fn new(name: String, entry: usize) -> Context {
        let mut arch = Box::new(ContextRegs::default());

        let kernel_stack = unsafe {
            FRAME_ALLOCATOR
                .lock()
                .allocate(FrameCount::new(
                    STACK_SIZE / crate::arch::CurrentMMArch::PAGE_SIZE,
                ))
                .map(|p| CurrentMMArch::phys_to_virt(p))
        }
        .expect("Cannot allocate kernel stack");

        arch.init(entry, kernel_stack.data() + STACK_SIZE);

        let context = Context {
            pid: NEXT_PID.fetch_add(1, Ordering::SeqCst),
            arch,
            name: name.clone(),
            kernel_stack,
            init_info: None,
            brk_start: BRK_START,
            brk_end: BRK_START,
            actions: BTreeMap::new(),
            signal: 0,
            blocked: 0,
            signal_stack: None,
            mapped_regions: Vec::new(),
        };

        init_file_descriptor_manager_with_stdin_stdout(
            context.get_pid(),
            StdioIndexNode::new(),
            StdioIndexNode::new(),
        );

        context
    }

    pub fn do_fork(&self, regs: usize) -> Result<usize> {
        let mut new = self.clone();

        new.arch.set_address(VirtualAddress::new(regs));

        Ok(new.pid)
    }

    pub fn get_pid(&self) -> usize {
        self.pid
    }

    pub fn arch(&self) -> &dyn ArchContext {
        self.arch.as_ref()
    }

    pub fn arch_mut(&mut self) -> &mut dyn ArchContext {
        self.arch.as_mut()
    }

    pub fn brk(&mut self, addr: usize) -> Result<usize> {
        let aligned_addr = (addr + CurrentMMArch::PAGE_SIZE - 1) & !CurrentMMArch::PAGE_OFFSET_MASK;

        if aligned_addr == 0 {
            return Ok(self.brk_start);
        } else if aligned_addr < self.brk_end {
            return Ok(0);
        }

        let addr = self.brk_end;
        let size = aligned_addr - self.brk_start;

        let count = (size + CurrentMMArch::PAGE_SIZE - 1) / CurrentMMArch::PAGE_SIZE;

        let mut mapper = unsafe { PageMapper::current(rmm::TableKind::User, TheFrameAllocator) };

        for i in 0..count {
            if let Some(flusher) = unsafe {
                mapper.map(
                    VirtualAddress::new(self.brk_end + i * CurrentMMArch::PAGE_SIZE),
                    PageFlags::new().write(true).user(true),
                )
            } {
                flusher.flush();
            }
        }

        self.brk_end = aligned_addr;

        Ok(self.brk_end)
    }

    pub fn find_free_area(&mut self, size: usize) -> VirtualAddress {
        const MMAP_AREA_START: usize = 0x6000_0000_0000;
        const MMAP_AREA_END: usize = 0x7000_0000_0000;

        let mut candidate = MMAP_AREA_START;

        let mut all_regions = alloc::vec![(self.brk_start, self.brk_end - self.brk_start)];
        all_regions.extend(&self.mapped_regions);
        all_regions.sort_by_key(|&(start, _)| start);

        for &(start, len) in &all_regions {
            let end = start + len;
            if end > candidate && start < MMAP_AREA_END {
                candidate = candidate.max(end);
            }
        }

        // 确保地址对齐
        let aligned_addr =
            (candidate + CurrentMMArch::PAGE_SIZE - 1) & !(CurrentMMArch::PAGE_SIZE - 1);

        if aligned_addr + size > MMAP_AREA_END {
            panic!("MMAP address space exhausted");
        }

        VirtualAddress::new(aligned_addr)
    }
}

impl Clone for Context {
    fn clone(&self) -> Self {
        let arch = self.arch.clone();

        let kernel_stack = unsafe {
            FRAME_ALLOCATOR
                .lock()
                .allocate(FrameCount::new(
                    STACK_SIZE / crate::arch::CurrentMMArch::PAGE_SIZE,
                ))
                .map(|p| CurrentMMArch::phys_to_virt(p))
        }
        .expect("Cannot allocate kernel stack");

        let context = Context {
            pid: NEXT_PID.fetch_add(1, Ordering::SeqCst),
            arch,
            name: self.name.clone(),
            kernel_stack,
            init_info: None,
            brk_start: BRK_START,
            brk_end: BRK_START,
            actions: BTreeMap::new(),
            signal: 0,
            blocked: 0,
            signal_stack: None,
            mapped_regions: Vec::new(),
        };

        init_file_descriptor_manager_with_stdin_stdout(
            context.get_pid(),
            StdioIndexNode::new(),
            StdioIndexNode::new(),
        );

        context
    }
}

pub type SharedContext = Arc<RwLock<Context>>;
pub type WeakSharedContext = Weak<RwLock<Context>>;

pub fn init() {
    SCHEDULER.lock().add(Arc::new(RwLock::new(Context::new(
        "init".to_string(),
        crate::init as usize,
    ))));
}
