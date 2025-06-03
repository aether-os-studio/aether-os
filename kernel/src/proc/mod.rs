use core::sync::atomic::{AtomicUsize, Ordering};

use alloc::collections::btree_map::BTreeMap;
use alloc::string::ToString;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use alloc::{boxed::Box, string::String};
use exec::ProcInitInfo;
use goblin::elf::ProgramHeaders;
use rmm::{Arch, FrameAllocator, FrameCount, PageFlags, PhysicalAddress, VirtualAddress};
use sched::get_current_context;
use spin::RwLock;

use crate::arch::CurrentMMArch;
use crate::arch::nr::{SigAction, SignalStack};
use crate::arch::rmm::PageMapper;
use crate::fs::*;
use crate::memory::frame::TheFrameAllocator;
use crate::proc::sched::SCHEDULER;
use crate::syscall::Result;
use crate::{arch::proc::context::ContextRegs, memory::FRAME_ALLOCATOR};
use fd::{
    FILE_DESCRIPTOR_MANAGERS, init_file_descriptor_manager,
    init_file_descriptor_manager_with_stdin_stdout,
};

pub mod abi;
pub mod exec;
pub mod sched;
pub mod signal;
pub mod syscall;

pub const MAX_FD_NUM: usize = 64;

pub trait ArchContext {
    /// 内核线程初始化
    fn init(&mut self, entry: usize, stack: usize);
    /// 前往用户态
    fn go_to_user(&mut self, entry: usize, stack: usize);
    /// 复制自己
    fn clone(&self, addr: VirtualAddress, source: VirtualAddress) -> Box<dyn ArchContext>;
    /// 获取地址
    fn address(&self) -> VirtualAddress;
    /// 设置寄存器地址
    fn set_address(&mut self, addr: VirtualAddress);
    /// 获取页表地址
    fn page_table_address(&self) -> PhysicalAddress;
    /// 切换到当前
    fn make_current(&self);
    /// 释放资源
    fn exit(&self);
    /// 设置架构信息
    #[cfg(target_arch = "x86_64")]
    fn get_fs(&self) -> usize;
    #[cfg(target_arch = "x86_64")]
    fn set_fs(&mut self, fsbase: usize);
}

pub const STACK_SIZE: usize = 65536;

#[derive(Default)]
pub struct ContextMemoryMappings {
    pub mapped_regions: Vec<(usize, usize)>,
    pub phdrs: ProgramHeaders,
    pub interpreter_load_start: usize,
    pub interpreter_load_end: usize,
    pub brk_start: usize,
    pub brk_end: usize,
}

/// 最小调度单位
pub struct Context {
    pid: usize,
    pub ppid: usize,
    arch: Box<dyn ArchContext>,
    name: String,
    kernel_stack: VirtualAddress,
    pub syscall_stack: VirtualAddress,
    init_info: Option<ProcInitInfo>,
    actions: BTreeMap<usize, SigAction>,
    signal: u64,
    blocked: u64,
    signal_stack: Option<SignalStack>,
    pub code: usize,
    pub memory_mappings: ContextMemoryMappings,
    pub termios: Termios,
}

pub const BRK_START: usize = 0x0000_7000_0000_0000;

unsafe impl Sync for Context {}
unsafe impl Send for Context {}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct PosixOldUtsName {
    pub sysname: [u8; 65],
    pub nodename: [u8; 65],
    pub release: [u8; 65],
    pub version: [u8; 65],
    pub machine: [u8; 65],
}

impl PosixOldUtsName {
    pub fn new() -> Self {
        const SYS_NAME: &[u8] = b"AetherOS";
        const NODENAME: &[u8] = b"AetherOS";
        const RELEASE: &[u8] = env!("CARGO_PKG_VERSION").as_bytes();
        const VERSION: &[u8] = env!("CARGO_PKG_VERSION").as_bytes();

        #[cfg(target_arch = "x86_64")]
        const MACHINE: &[u8] = b"x86_64";

        #[cfg(target_arch = "aarch64")]
        const MACHINE: &[u8] = b"aarch64";

        #[cfg(target_arch = "riscv64")]
        const MACHINE: &[u8] = b"riscv64";

        #[cfg(target_arch = "loongarch64")]
        const MACHINE: &[u8] = b"longarch64";

        let mut r = Self {
            sysname: [0; 65],
            nodename: [0; 65],
            release: [0; 65],
            version: [0; 65],
            machine: [0; 65],
        };

        r.sysname[0..SYS_NAME.len()].copy_from_slice(SYS_NAME);
        r.nodename[0..NODENAME.len()].copy_from_slice(NODENAME);
        r.release[0..RELEASE.len()].copy_from_slice(RELEASE);
        r.version[0..VERSION.len()].copy_from_slice(VERSION);
        r.machine[0..MACHINE.len()].copy_from_slice(MACHINE);

        return r;
    }
}

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

        let syscall_stack = unsafe {
            FRAME_ALLOCATOR
                .lock()
                .allocate(FrameCount::new(
                    STACK_SIZE / crate::arch::CurrentMMArch::PAGE_SIZE,
                ))
                .map(|p| CurrentMMArch::phys_to_virt(p))
        }
        .expect("Cannot allocate kernel stack");

        arch.init(entry, kernel_stack.data() + STACK_SIZE);

        let mut context = Context {
            pid: NEXT_PID.fetch_add(1, Ordering::SeqCst),
            ppid: 0,
            arch,
            name: name.clone(),
            kernel_stack: kernel_stack.add(STACK_SIZE),
            syscall_stack: syscall_stack.add(STACK_SIZE),
            init_info: None,
            actions: BTreeMap::new(),
            signal: 0,
            blocked: 0,
            signal_stack: None,
            code: 0,
            memory_mappings: Default::default(),
            termios: Default::default(),
        };

        context.termios.c_iflag =
            BRKINT as u32 | ICRNL as u32 | INPCK as u32 | ISTRIP as u32 | IXON as u32;
        context.termios.c_oflag = OPOST as u32;
        context.termios.c_cflag = CS8 as u32 | CREAD as u32 | CLOCAL as u32;
        context.termios.c_lflag = ECHO as u32 | ICANON as u32 | IEXTEN as u32 | ISIG as u32;
        context.termios.c_line = 0;
        context.termios.c_cc[VINTR] = 3; // Ctrl-C
        context.termios.c_cc[VQUIT] = 28; // Ctrl-context.termios.c_cc[VERASE] = 127; // DEL
        context.termios.c_cc[VKILL] = 21; // Ctrl-U
        context.termios.c_cc[VEOF] = 4; // Ctrl-D
        context.termios.c_cc[VTIME] = 0; // No timer
        context.termios.c_cc[VMIN] = 1; // Return each byte
        context.termios.c_cc[VSTART] = 17; // Ctrl-Q
        context.termios.c_cc[VSTOP] = 19; // Ctrl-S
        context.termios.c_cc[VSUSP] = 26; // Ctrl-Z
        context.termios.c_cc[VREPRINT] = 18; // Ctrl-R
        context.termios.c_cc[VDISCARD] = 15; // Ctrl-O
        context.termios.c_cc[VWERASE] = 23; // Ctrl-W
        context.termios.c_cc[VLNEXT] = 22; // Ctrl-V

        context.memory_mappings.brk_start = BRK_START;
        context.memory_mappings.brk_end = BRK_START;

        init_file_descriptor_manager_with_stdin_stdout(
            context.get_pid(),
            StdioIndexNode::new(),
            StdioIndexNode::new(),
        );

        context
    }

    pub fn do_fork(&self, regs: usize) -> Result<usize> {
        let new = self.full_clone(regs);

        Ok(new.read().pid)
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
            return Ok(self.memory_mappings.brk_start);
        } else if aligned_addr < self.memory_mappings.brk_end {
            return Ok(0);
        }

        let addr = self.memory_mappings.brk_end;
        let size = aligned_addr - self.memory_mappings.brk_start;

        let count = (size + CurrentMMArch::PAGE_SIZE - 1) / CurrentMMArch::PAGE_SIZE;

        let mut mapper = unsafe { PageMapper::current(rmm::TableKind::User, TheFrameAllocator) };

        for i in 0..count {
            if let Some(flusher) = unsafe {
                mapper.map(
                    VirtualAddress::new(
                        self.memory_mappings.brk_end + i * CurrentMMArch::PAGE_SIZE,
                    ),
                    PageFlags::new().write(true).user(true),
                )
            } {
                flusher.flush();
            }
        }

        self.memory_mappings.brk_end = aligned_addr;

        Ok(self.memory_mappings.brk_end)
    }

    pub fn find_free_area(&mut self, size: usize) -> VirtualAddress {
        const MMAP_AREA_START: usize = 0x6000_0000_0000;
        const MMAP_AREA_END: usize = 0x7000_0000_0000;

        let mut candidate = MMAP_AREA_START;

        let mut all_regions = alloc::vec![(
            self.memory_mappings.brk_start,
            self.memory_mappings.brk_end - self.memory_mappings.brk_start
        )];
        all_regions.extend(&self.memory_mappings.mapped_regions);
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

    pub fn full_clone(&self, regs_ptr: usize) -> SharedContext {
        let kernel_stack = unsafe {
            FRAME_ALLOCATOR
                .lock()
                .allocate(FrameCount::new(
                    STACK_SIZE / crate::arch::CurrentMMArch::PAGE_SIZE,
                ))
                .map(|p| CurrentMMArch::phys_to_virt(p))
        }
        .expect("Cannot allocate kernel stack");

        let syscall_stack = unsafe {
            FRAME_ALLOCATOR
                .lock()
                .allocate(FrameCount::new(
                    STACK_SIZE / crate::arch::CurrentMMArch::PAGE_SIZE,
                ))
                .map(|p| CurrentMMArch::phys_to_virt(p))
        }
        .expect("Cannot allocate syscall stack");

        let arch = self
            .arch
            .clone(kernel_stack.add(STACK_SIZE), VirtualAddress::new(regs_ptr));

        let mut context = Context {
            pid: NEXT_PID.fetch_add(1, Ordering::SeqCst),
            ppid: get_current_context().read().get_pid(),
            arch,
            name: self.name.clone(),
            kernel_stack: kernel_stack.add(STACK_SIZE),
            syscall_stack: syscall_stack.add(STACK_SIZE),
            init_info: None,
            actions: BTreeMap::new(),
            signal: 0,
            blocked: 0,
            signal_stack: None,
            code: 0,
            memory_mappings: Default::default(),
            termios: Default::default(),
        };

        context.termios.c_iflag =
            BRKINT as u32 | ICRNL as u32 | INPCK as u32 | ISTRIP as u32 | IXON as u32;
        context.termios.c_oflag = OPOST as u32;
        context.termios.c_cflag = CS8 as u32 | CREAD as u32 | CLOCAL as u32;
        context.termios.c_lflag = ECHO as u32 | ICANON as u32 | IEXTEN as u32 | ISIG as u32;
        context.termios.c_line = 0;
        context.termios.c_cc[VINTR] = 3; // Ctrl-C
        context.termios.c_cc[VQUIT] = 28; // Ctrl-context.termios.c_cc[VERASE] = 127; // DEL
        context.termios.c_cc[VKILL] = 21; // Ctrl-U
        context.termios.c_cc[VEOF] = 4; // Ctrl-D
        context.termios.c_cc[VTIME] = 0; // No timer
        context.termios.c_cc[VMIN] = 1; // Return each byte
        context.termios.c_cc[VSTART] = 17; // Ctrl-Q
        context.termios.c_cc[VSTOP] = 19; // Ctrl-S
        context.termios.c_cc[VSUSP] = 26; // Ctrl-Z
        context.termios.c_cc[VREPRINT] = 18; // Ctrl-R
        context.termios.c_cc[VDISCARD] = 15; // Ctrl-O
        context.termios.c_cc[VWERASE] = 23; // Ctrl-W
        context.termios.c_cc[VLNEXT] = 22; // Ctrl-V

        context.memory_mappings.brk_start = BRK_START;
        context.memory_mappings.brk_end = BRK_START;

        init_file_descriptor_manager(context.get_pid());

        let file_descriptor_manager = FILE_DESCRIPTOR_MANAGERS.lock();

        let context_file_descriptor_manager = file_descriptor_manager.get(&context.pid).unwrap();

        for (_, (inode, mode, _)) in file_descriptor_manager
            .get(&self.pid)
            .unwrap()
            .file_descriptors
            .iter()
        {
            context_file_descriptor_manager.add_inode(inode.clone(), mode.clone());
        }

        let context = Arc::new(RwLock::new(context));

        SCHEDULER.lock().add(context.clone());

        context
    }
}

impl Drop for Context {
    fn drop(&mut self) {
        self.arch.exit();
    }
}

pub type SharedContext = Arc<RwLock<Context>>;
pub type WeakSharedContext = Weak<RwLock<Context>>;

pub fn init() {
    sched::init();

    SCHEDULER.lock().add(Arc::new(RwLock::new(Context::new(
        "init".to_string(),
        crate::init as usize,
    ))));
}
