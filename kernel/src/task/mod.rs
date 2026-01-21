use core::sync::atomic::AtomicUsize;

use alloc::{
    collections::vec_deque::VecDeque,
    string::{String, ToString},
    sync::{Arc, Weak},
    vec::Vec,
};
use rmm::{Arch, FrameAllocator, FrameCount, PhysicalAddress, VirtualAddress};
use spin::{Mutex, RwLock};

#[cfg(any(target_arch = "x86_64", target_arch = "riscv64"))]
use crate::init::memory::KERNEL_PAGE_TABLE_PHYS;
use crate::{
    arch::{ArchContext, CurrentRmmArch, Ptrace, get_archid, irq::IrqRegsArch, switch_to},
    consts::STACK_SIZE,
    fs::ROOT_FILE_SYSTEM,
    init::memory::{FRAME_ALLOCATOR, PAGE_SIZE},
    initial_kernel_thread,
    smp::{CPU_COUNT, get_archid_by_cpuid, get_cpuid_by_archid},
    task::{
        sched::{ArcScheduler, SCHEDULERS},
        user::create_user_task,
    },
};

pub mod sched;
pub mod user;

pub type ArcTask = Arc<RwLock<Task>>;
pub type WeakArcTask = Weak<RwLock<Task>>;

pub static NEXT_PID: AtomicUsize = AtomicUsize::new(0);

pub struct Task {
    pub syscall_stack_top: VirtualAddress,
    pub user_syscall_stack: VirtualAddress,
    kernel_stack_top: VirtualAddress,
    name: String,
    parent: Option<WeakArcTask>,
    pid: usize,
    cpu_id: usize,
    pub arch_context: ArchContext,
    pub page_table_addr: PhysicalAddress,
}

pub const IDLE_PRIORITY: usize = 20;
pub const NORMAL_PRIORITY: usize = 0;

fn alloc_cpuid() -> usize {
    static NEXT_CPUID: AtomicUsize = AtomicUsize::new(0);
    let cpu_count = CPU_COUNT.load(core::sync::atomic::Ordering::SeqCst);
    let mut next = NEXT_CPUID.load(core::sync::atomic::Ordering::SeqCst);
    let ret = next;
    next = (next + 1) % cpu_count;
    NEXT_CPUID.store(next, core::sync::atomic::Ordering::SeqCst);
    ret
}

impl Task {
    pub fn new_idle(cpu_id: usize) -> ArcTask {
        Self::new_inner(0, cpu_id, "idle".to_string(), IDLE_PRIORITY)
    }

    pub fn new(name: String) -> ArcTask {
        let pid = NEXT_PID.fetch_add(1, core::sync::atomic::Ordering::SeqCst);
        let cpu_id = alloc_cpuid();
        Self::new_inner(pid, cpu_id, name, NORMAL_PRIORITY)
    }

    fn new_inner(pid: usize, cpu_id: usize, name: String, _priority: usize) -> ArcTask {
        let stack_frame_count = FrameCount::new(STACK_SIZE / PAGE_SIZE);
        let kernel_stack_phys = unsafe { FRAME_ALLOCATOR.lock().allocate(stack_frame_count) }
            .expect("No memory to allocate kernel stack");
        let kernel_stack_virt = unsafe { CurrentRmmArch::phys_to_virt(kernel_stack_phys) };
        let syscall_stack_phys = unsafe { FRAME_ALLOCATOR.lock().allocate(stack_frame_count) }
            .expect("No memory to allocate syscall stack");
        let syscall_stack_virt = unsafe { CurrentRmmArch::phys_to_virt(syscall_stack_phys) };

        let task = Task {
            syscall_stack_top: syscall_stack_virt.add(STACK_SIZE),
            user_syscall_stack: VirtualAddress::new(0),
            kernel_stack_top: kernel_stack_virt.add(STACK_SIZE),
            name,
            parent: get_current_task().map(|t| Arc::downgrade(&t)),
            pid,
            cpu_id,
            arch_context: ArchContext::default(),
            #[cfg(any(target_arch = "x86_64", target_arch = "riscv64"))]
            page_table_addr: PhysicalAddress::new(
                KERNEL_PAGE_TABLE_PHYS.load(core::sync::atomic::Ordering::SeqCst),
            ),
            #[cfg(any(target_arch = "aarch64", target_arch = "loongarch64"))]
            page_table_addr: PhysicalAddress::new(0),
        };
        Arc::new(RwLock::new(task))
    }

    pub fn get_name(&self) -> String {
        self.name.clone()
    }

    pub fn get_parent(&self) -> Option<ArcTask> {
        self.parent.clone()?.upgrade()
    }

    pub fn get_pid(&self) -> usize {
        self.pid
    }

    pub fn get_cpu_id(&self) -> usize {
        self.cpu_id
    }

    pub fn get_kernel_stack_top(&self) -> VirtualAddress {
        self.kernel_stack_top
    }

    pub fn pt_regs(&self) -> *mut Ptrace {
        unsafe { (self.kernel_stack_top.data() as *mut Ptrace).sub(1) }
    }

    pub fn set_kernel_context_info(&mut self, entry: usize, stack_top: VirtualAddress) {
        let regs = unsafe { self.pt_regs().as_mut_unchecked() };
        regs.set_user_space(false);
        regs.set_args((0, 0, entry as u64, 0, 0, 0));
        self.arch_context.ip = crate::arch::kernel_thread_entry as *const () as usize;
        self.arch_context.sp = stack_top.data();
    }

    pub fn set_user_context_info(
        &mut self,
        entry: usize,
        stack_top: VirtualAddress,
        args: Option<(usize, usize, usize, usize, usize, usize)>,
    ) {
        self.arch_context.ip = crate::arch::return_from_interrupt as *const () as usize;
        self.arch_context.sp = self.pt_regs() as usize;
        let regs = unsafe { self.pt_regs().as_mut_unchecked() };
        regs.set_user_space(true);
        regs.set_ip(entry as u64);
        regs.set_sp(stack_top.data() as u64);
        if let Some(args) = args {
            let args = (
                args.0 as u64,
                args.1 as u64,
                args.2 as u64,
                args.3 as u64,
                args.4 as u64,
                args.5 as u64,
            );
            regs.set_args(args);
        }
    }
}

impl Drop for Task {
    fn drop(&mut self) {
        let mut frame_allocator = FRAME_ALLOCATOR.lock();
        let stack_frame_count = FrameCount::new(STACK_SIZE / PAGE_SIZE);
        let kernel_stack_phys = PhysicalAddress::new(
            self.kernel_stack_top
                .sub(CurrentRmmArch::PHYS_OFFSET)
                .data(),
        )
        .sub(STACK_SIZE);
        unsafe { frame_allocator.free(kernel_stack_phys, stack_frame_count) };
        let syscall_stack_phys = PhysicalAddress::new(
            self.syscall_stack_top
                .sub(CurrentRmmArch::PHYS_OFFSET)
                .data(),
        )
        .sub(STACK_SIZE);
        unsafe { frame_allocator.free(syscall_stack_phys, stack_frame_count) };
    }
}

pub static TASKS: Mutex<VecDeque<ArcTask>> = Mutex::new(VecDeque::new());

pub fn add_task(task: ArcTask) {
    let archid = get_archid_by_cpuid(task.read().get_cpu_id());
    get_scheduler_by_archid(archid)
        .write()
        .add_task(task.clone());
    TASKS.lock().push_back(task);
}

pub fn get_scheduler_by_archid(archid: usize) -> ArcScheduler {
    SCHEDULERS.lock().get(&archid).unwrap().clone()
}

pub fn get_scheduler_by_cpuid(cpuid: usize) -> ArcScheduler {
    SCHEDULERS
        .lock()
        .get(&get_cpuid_by_archid(cpuid))
        .unwrap()
        .clone()
}

pub fn get_scheduler() -> ArcScheduler {
    get_scheduler_by_archid(get_archid())
}

pub fn get_current_task() -> Option<ArcTask> {
    get_scheduler().read().get_current_task()
}

pub fn create_kernel_task(name: String, entry: usize) -> Option<ArcTask> {
    let task = Task::new(name);
    let mut task_guard = task.write();
    let stack_top = VirtualAddress::new(task_guard.pt_regs() as usize);
    task_guard.set_kernel_context_info(entry, stack_top);
    drop(task_guard);
    add_task(task.clone());
    Some(task)
}

pub fn block(task: ArcTask) {
    let scheduler = get_scheduler_by_cpuid(task.read().cpu_id);
    scheduler.write().remove_task(task);
}

pub fn unblock(task: ArcTask) {
    let scheduler = get_scheduler_by_cpuid(task.read().cpu_id);
    scheduler.write().add_task(task);
}

pub fn cleanup(task: ArcTask) {
    let mut tasks = TASKS.lock();
    let pos = tasks.iter().position(|p| Arc::ptr_eq(p, &task)).unwrap();
    tasks.remove(pos);
}

pub fn exit_current() -> ! {
    block(get_current_task().unwrap());
    loop {
        schedule();
    }
}

pub fn init() -> Option<ArcTask> {
    for cpu_id in 0..CPU_COUNT.load(core::sync::atomic::Ordering::SeqCst) {
        let idle_task = Task::new_idle(cpu_id);
        TASKS.lock().push_back(idle_task.clone());
        let scheduler = get_scheduler_by_cpuid(cpu_id);
        let mut scheduler = scheduler.write();
        scheduler.set_current_task(idle_task);
    }

    create_kernel_task(
        "init".to_string(),
        initial_kernel_thread as *const () as usize,
    )
}

pub fn init_user() -> Option<ArcTask> {
    let init_file = ROOT_FILE_SYSTEM
        .write()
        .lookup("/sbin/init".to_string(), true)?;
    create_user_task("init".to_string(), init_file, Vec::new(), Vec::new()).ok()
}

pub fn schedule() {
    let current_scheduler = get_scheduler();
    let prev = current_scheduler
        .read()
        .get_current_task()
        .expect("Scheduler not initialized");
    let next = current_scheduler.write().schedule();
    drop(current_scheduler);
    switch_to(prev, next);
}
