#include <task/task.h>
#include <mm/hhdm.h>
#include <mm/frame.h>
#include <mm/page.h>
#include <mm/heap.h>
#include <irq/gate.h>
#include <irq/irq.h>
#include <kprint.h>

bool can_schedule;

struct List block_list;

tss_t tss[MAX_CPU_NUM];

task_t *tasks[MAX_TASK_NUM];
task_t *idle_tasks[MAX_CPU_NUM];

spinlock_t fork_lock;

hardware_intr_controller apic_timer_controller = {
    .install = ioapic_add,
};

extern uint64_t cpu_count;
static volatile uint32_t cpu_idx = 0;
spinlock_t cpu_alloc_lock;

uint32_t alloc_cpuid()
{
    spin_lock(&cpu_alloc_lock);
    uint32_t idx = cpu_idx;
    cpu_idx = (cpu_idx + 1) % cpu_count;
    spin_unlock(&cpu_alloc_lock);
    return idx;
}

task_t *get_free_task()
{
    for (uint64_t i = 0; i < MAX_TASK_NUM; i++)
    {
        if (tasks[i] == NULL)
        {
            task_t *task = (task_t *)phys_to_virt(alloc_frames(1));
            memset(task, 0, PAGE_SIZE);
            task->task_id = i;
            tasks[i] = task;
            return task;
        }
    }

    return NULL;
}

task_t *task_search(task_state_t state, uint32_t cpu_id)
{
    task_t *task = NULL;

    for (size_t i = cpu_count; i < MAX_TASK_NUM; i++)
    {
        task_t *ptr = tasks[i];
        if (ptr == NULL)
            continue;
        if (current_task == ptr)
            continue;
        if (ptr->state != state)
            continue;
        if (ptr->on_cpu != cpu_id)
            continue;

        if (task == NULL || ptr->jiffies < task->jiffies)
            task = ptr;
    }

    if (task == NULL && state == TASK_READY)
    {
        task = idle_tasks[cpu_id];
    }

    return task;
}

void timer_handler(uint8_t irq, uint64_t param, struct pt_regs *regs)
{
    current_task->jiffies++;

    current_task->need_schedule = true;
}

task_t *task_create(const char *name, void (*entry)(), uint64_t uid)
{
    task_t *task = get_free_task();

    uint64_t stack = (uint64_t)phys_to_virt(alloc_frames(STACK_SIZE / PAGE_SIZE)) + STACK_SIZE;
    stack -= sizeof(struct pt_regs);
    struct pt_regs *task_frame = (struct pt_regs *)stack;

    memset(task_frame, 0, sizeof(struct pt_regs));
    task_frame->rflags = (1UL << 9);
    task_frame->rbp = stack;
    task_frame->rsp = stack;
    task_frame->rip = (uint64_t)entry;
    task_frame->cs = SELECTOR_KERNEL_CS;
    task_frame->ss = SELECTOR_KERNEL_DS;
    task_frame->ds = SELECTOR_KERNEL_DS;
    task_frame->es = SELECTOR_KERNEL_DS;
    task->context = task_frame;

    strcpy(task->name, name);
    task->self_ref = (uint64_t)task;
    list_init(&task->list);
    task->parent_task_id = task->task_id;
    task->waitpid = 0;
    task->syscall_stack = phys_to_virt(alloc_frames(STACK_SIZE / PAGE_SIZE)) + STACK_SIZE;
    task->on_cpu = alloc_cpuid();
    task->pgdir = get_kernel_page_dir();
    task->state = TASK_READY;
    task->fpu = (fpu_context *)phys_to_virt(alloc_frames(1));
    memset(task->fpu, 0, PAGE_SIZE);
    task->fpu->mxscr = 0x1f80;
    task->fpu->fcw = 0x037f;

    task->thread = (task_thread_t *)(task + 1);
    task->thread->fs = SELECTOR_KERNEL_DS;
    task->thread->gs = SELECTOR_KERNEL_DS;
    task->thread->fsbase = 0;
    task->thread->gsbase = 0;
    task->thread->elf_mapping_start = 0;
    task->thread->elf_mapping_end = 0;

    task->signal = 0;
    task->blocked = 0;

    task->need_schedule = false;
    task->userspace_io_allowed = false;

    task->brk_start = USER_BRK_START;
    task->brk_end = USER_BRK_START;

    task->uid = uid;

    task->status = 0;
    task->state = TASK_READY;

    return task;
}

void idle_thread()
{
    while (1)
    {
        __asm__ __volatile__("sti\n\thlt\n\t");
    }
}

extern void init_module();

void init_thread()
{
    kdebug("init thread is running");

    init_module();

    kerror("load module failed");

    while (1)
    {
        sti();
        hlt();
    }
}

void tss_init()
{
    uint64_t sp = phys_to_virt(alloc_frames(STACK_SIZE / PAGE_SIZE)) + STACK_SIZE;
    uint64_t offset = 10 + current_cpu_id * 2;
    set_tss64((uint32_t *)&tss[current_cpu_id], sp, sp, sp, sp, sp, sp, sp, sp, sp, sp);
    set_tss_descriptor(offset, &tss[current_cpu_id]);
    load_TR(offset);
}

bool task_initialized = false;

void task_init()
{
    memset(tasks, 0, sizeof(tasks));
    memset(idle_tasks, 0, sizeof(idle_tasks));

    spin_init(&cpu_alloc_lock);
    spin_init(&fork_lock);
    list_init(&block_list);

    for (uint64_t i = 0; i < cpu_count; i++)
    {
        idle_tasks[i] = task_create("idle", idle_thread, KERNEL_USER);
    }
    task_create("init", init_thread, NORMAL_USER);

    irq_register(APIC_TIMER_INTERRUPT_VECTOR, timer_handler, 0, &apic_timer_controller, "APIC TIMER");

    task_initialized = true;

    can_schedule = true;

    cli();

    write_kgsbase((uint64_t)idle_tasks[current_cpu_id]);

    task_switch_to(NULL, NULL, idle_tasks[current_cpu_id]);
}

void task_switch_to(struct pt_regs *curr, task_t *prev, task_t *next)
{
    if (curr != NULL && prev != NULL)
    {
        memcpy(prev->context, curr, sizeof(struct pt_regs));
        // prev->context = curr;
    }

    if (prev)
    {
        __asm__ __volatile__("movq %%fs, %0\n\t" : "=r"(prev->thread->fs));
        __asm__ __volatile__("movq %%gs, %0\n\t" : "=r"(prev->thread->gs));

        prev->thread->fsbase = read_fsbase();
        prev->thread->gsbase = read_gsbase();

        prev->state = TASK_READY;

        __asm__ __volatile__("fxsave (%0)" ::"r"(prev->fpu));
    }

    if (!next)
        kerror("next should not be NULL");

    if (can_schedule)
    {
        // Start to switch
        tss[current_cpu_id].rsp0 = (uint64_t)(next->context + 1);
        uint16_t value = next->userspace_io_allowed ? offsetof(tss_t, iomap) : 0xFFFF;
        tss[current_cpu_id].iomapbaseaddr = value;

        __asm__ __volatile__("movq %0, %%fs\n\t" ::"r"(next->thread->fs));
        __asm__ __volatile__("movq %0, %%gs\n\t" ::"r"(next->thread->gs));

        write_fsbase(next->thread->fsbase);
        write_gsbase(next->thread->gsbase);

        next->state = TASK_RUNNING;

        __asm__ __volatile__("fxrstor (%0)" ::"r"(next->fpu));

        __asm__ __volatile__("movq %0, %%cr3\n\t" ::"r"(next->pgdir->table));

        write_kgsbase((uint64_t)next);

        __asm__ __volatile__(
            "movq %0, %%rsp\n\t"
            "jmp ret_from_intr" ::"r"(next->context));
    }
}

void task_exit(int code)
{
    if (current_task->state != TASK_RUNNING)
    {
        return;
    }

    // kinfo("task %s %#018lx exit......", current_task->name, current_task);

    current_task->state = TASK_DIED;

    free_frames(virt_to_phys((uint64_t)current_task->fpu), 1);

    current_task->status = code;

    task_t *parent = tasks[current_task->parent_task_id];
    if (parent->state == TASK_BLOCKING && (parent->waitpid == 0 || parent->waitpid == current_task->task_id))
    {
        task_unblock(parent, EOK);
    }

    task_t *next = task_search(TASK_READY, current_cpu_id);

    task_switch_to(NULL, NULL, next);
}

task_t *get_task(uint64_t pid)
{
    for (uint64_t i = 0; i < MAX_TASK_NUM; i++)
    {
        if (tasks[i] == NULL)
            continue;
        if (tasks[i]->task_id == pid)
            return tasks[i];
    }
}

int task_block(task_t *task, struct List *blist, task_state_t state, int timeout_ms)
{
    if (blist == NULL)
    {
        blist = &block_list;
    }

    list_add_to_behind(blist, &task->list);
    if (timeout_ms > 0)
    {
        // todo
    }

    task->state = state;

    if (current_task == task)
    {
        sti();

        __asm__ __volatile__("int %0\n\t" ::"i"(APIC_TIMER_INTERRUPT_VECTOR));
    }

    return task->status;
}

void task_unblock(task_t *task, int reason)
{
    if (task->list.next)
    {
        list_del(&task->list);
    }

    task->status = reason;
    task->state = TASK_READY;
}

uint64_t sys_fork(struct pt_regs *regs)
{
    can_schedule = false;

    spin_lock(&fork_lock);

    task_t *child = get_free_task();
    strcpy(child->name, current_task->name);

    child->self_ref = (uint64_t)child;
    list_init(&child->list);
    child->on_cpu = alloc_cpuid();

    child->uid = current_task->uid;

    child->parent_task_id = current_task->task_id;

    child->state = TASK_READY;
    child->fpu = (fpu_context *)phys_to_virt(alloc_frames(1));
    memcpy(child->fpu, current_task->fpu, PAGE_SIZE);

    child->pgdir = clone_directory(get_current_page_dir());

    child->syscall_stack = phys_to_virt(alloc_frames(STACK_SIZE / PAGE_SIZE)) + STACK_SIZE;

    child->thread = (task_thread_t *)(child + 1);
    memcpy(child->thread, current_task->thread, sizeof(task_thread_t));

    uint64_t stack = (uint64_t)phys_to_virt(alloc_frames(STACK_SIZE / PAGE_SIZE)) + STACK_SIZE;
    stack -= sizeof(struct pt_regs);
    struct pt_regs *task_frame = (struct pt_regs *)stack;
    memcpy(task_frame, regs, sizeof(struct pt_regs));
    task_frame->rax = 0;

    child->context = task_frame;

    child->status = 0;

    child->signal = 0;
    child->blocked = 0;

    child->need_schedule = false;

    child->brk_start = USER_BRK_START;
    child->brk_end = USER_BRK_START;

    spin_unlock(&fork_lock);

    can_schedule = true;

    return child->task_id;
}

uint64_t sys_waitpid(uint64_t pid, int *status)
{
    task_t *child = NULL;

    while (1)
    {
        bool has_child = false;

        for (uint64_t i = cpu_count; i < MAX_TASK_NUM; i++)
        {
            task_t *ptr = tasks[i];
            if (ptr == NULL)
                continue;

            if (ptr->parent_task_id != current_task->task_id)
                continue;

            if (pid != ptr->task_id && pid != 0)
                continue;

            if (ptr->state == TASK_DIED)
            {
                child = ptr;
                tasks[i] = NULL;
                goto rollback;
            }

            has_child = true;
        }
        if (has_child)
        {
            current_task->waitpid = pid;
            task_block(current_task, NULL, TASK_BLOCKING, 0);
            continue;
        }

        break;
    }

    return -1;

rollback:
    *status = (int)child->status;
    uint32_t ret = child->task_id;

    free_page_table_recursive(child->pgdir, 4);

    free_frames((uint64_t)child, 1);

    return ret;
}

void sys_iopl(uint64_t level)
{
    if (level > 3)
    {
        return;
    }

    bool allow = (level == 3);

    current_task->userspace_io_allowed = allow;

    uint16_t value = allow ? offsetof(tss_t, iomap) : 0xFFFF;

    tss[current_cpu_id].iomapbaseaddr = value;
}
