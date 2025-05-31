use core::sync::atomic::{AtomicUsize, Ordering};

use alloc::string::ToString;
use alloc::sync::{Arc, Weak};
use alloc::{boxed::Box, string::String};
use exec::ProcInitInfo;
use rmm::{Arch, FrameAllocator, FrameCount, PhysicalAddress, VirtualAddress};
use sched::SCHEDULER;
use spin::RwLock;

use crate::arch::CurrentMMArch;
use crate::syscall::Result;
use crate::{arch::proc::context::ContextRegs, memory::FRAME_ALLOCATOR};

pub mod exec;
pub mod sched;

pub trait ArchContext {
    /// 内核线程初始化
    fn init(&mut self, entry: usize, stack: usize);
    /// 前往用户态
    fn go_to_user(&mut self);
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
}

pub const STACK_SIZE: usize = 32768;

/// 最小调度单位
pub struct Context {
    pid: usize,
    arch: Box<dyn ArchContext>,
    name: String,
    kernel_stack: VirtualAddress,
    init_info: Option<ProcInitInfo>,
}

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

        Context {
            pid: NEXT_PID.fetch_add(1, Ordering::SeqCst),
            arch,
            name: name.clone(),
            kernel_stack,
            init_info: None,
        }
    }

    pub fn do_fork(&self, regs: usize) -> Result<usize> {
        let mut new = self.clone();

        new.arch.set_address(VirtualAddress::new(regs));

        Ok(new.pid)
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

        Context {
            pid: NEXT_PID.fetch_add(1, Ordering::SeqCst),
            arch,
            name: self.name.clone(),
            kernel_stack,
            init_info: None,
        }
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
