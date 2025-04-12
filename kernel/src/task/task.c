#include <task/task.h>
#include <mm/hhdm.h>
#include <mm/frame.h>
#include <mm/page.h>
#include <irq/gate.h>
#include <irq/irq.h>
#include <kprint.h>

tss_t tss[MAX_CPU_NUM];

task_t *tasks[MAX_TASK_NUM];
task_t *idle_tasks[MAX_CPU_NUM];

extern uint64_t cpu_count;

uint32_t alloc_cpuid()
{
    static uint32_t i = 0;
    uint32_t ret = i++;
    if (i >= cpu_count)
        i = 0;
    return ret;
}

task_t *get_free_task()
{
    for (uint64_t i = 0; i < MAX_TASK_NUM; i++)
    {
        if (tasks[i] == NULL)
        {
            task_t *task = (task_t *)phys_to_virt(alloc_frames(1));
            memset(task, 0, sizeof(task_t));
            task->task_id = i;
            tasks[i] = task;
            return task;
        }
    }

    return NULL;
}

static task_t *task_search(task_state_t state, uint32_t cpu_id)
{
    task_t *task = NULL;

    for (size_t i = 0; i < MAX_TASK_NUM; i++)
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

    task_t *next = task_search(TASK_READY, current_cpu_id);

    task_switch_to(regs, current_task, next);
}

task_t *task_create(const char *name, void (*entry)())
{
    task_t *task = get_free_task();

    uint64_t stack = (uint64_t)task + PAGE_SIZE;

    stack -= sizeof(struct pt_regs);
    struct pt_regs *task_frame = (struct pt_regs *)stack;

    memset(task_frame, 0, sizeof(struct pt_regs));
    task_frame->rflags = 0x200;
    task_frame->rsp = stack;
    task_frame->rip = (uint64_t)entry;
    task_frame->cs = SELECTOR_KERNEL_CS;
    task_frame->ss = SELECTOR_KERNEL_DS;
    task_frame->ds = SELECTOR_KERNEL_DS;
    task_frame->es = SELECTOR_KERNEL_DS;
    task->context = task_frame;

    strcpy(task->name, name);
    task->self_ref = (uint64_t)task;
    task->on_cpu = alloc_cpuid();
    task->pgdir = clone_directory(get_kernel_page_dir());
    if (task->pgdir == NULL)
    {
        kerror("task %s alloc page dir failed", task->name);
        task->pgdir = get_kernel_page_dir();
    }
    task->state = TASK_READY;
    task->magic = AETHER_MAGIC;

    task->thread = (task_thread_t *)(task + 1);
    task->thread->fs = SELECTOR_KERNEL_DS;
    task->thread->gs = SELECTOR_KERNEL_DS;
    task->thread->fsbase = 0;
    task->thread->gsbase = 0;

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
    uint64_t sp = phys_to_virt(alloc_frames(1));
    uint64_t offset = 10 + current_cpu_id * 2;
    set_tss64((uint32_t *)&tss[current_cpu_id], sp, sp, sp, sp, sp, sp, sp, sp, sp, sp);
    set_tss_descriptor(offset, &tss[current_cpu_id]);
    load_TR(offset);
}

bool task_initialized = false;

void task_init()
{
    memset(tasks, 0, sizeof(tasks));
    for (uint64_t i = 0; i < cpu_count; i++)
    {
        idle_tasks[i] = task_create("idle", idle_thread);
    }
    task_create("init", init_thread);

    write_kgsbase((uint64_t)idle_tasks[current_cpu_id]);

    irq_register(APIC_TIMER_INTERRUPT_VECTOR, timer_handler, 0, NULL, "APIC TIMER");

    task_initialized = true;

    cli();

    task_switch_to(NULL, NULL, idle_tasks[current_cpu_id]);
}

void task_switch_to(struct pt_regs *curr, task_t *prev, task_t *next)
{
    if (curr != NULL && prev != NULL)
    {
        memcpy(prev->context, curr, sizeof(struct pt_regs));
    }

    if (next->magic == AETHER_MAGIC)
    {
        if (prev)
        {
            __asm__ __volatile__("movq %%fs, %0\n\t"
                                 "movq %%gs, %0\n\t" : "=r"(prev->thread->fs),
                                                       "=r"(prev->thread->gs));

            prev->thread->fsbase = read_fsbase();
            prev->thread->gsbase = read_gsbase();

            prev->state = TASK_READY;
        }

        // Start to schedule
        write_kgsbase((uint64_t)next);

        __asm__ __volatile__("movq %0, %%fs\n\t"
                             "movq %0, %%gs\n\t" ::"r"(next->thread->fs),
                             "r"(next->thread->gs));

        write_fsbase(next->thread->fsbase);
        write_gsbase(next->thread->gsbase);

        next->state = TASK_RUNNING;
        __asm__ __volatile__("movq %0, %%cr3\n\t" ::"r"(next->pgdir->table));
        __asm__ __volatile__("movq %0, %%rsp\n\t"
                             "jmp ret_from_intr" ::"r"(next->context));
    }
}

void task_exit(int code)
{
    if (current_task->state != TASK_RUNNING)
    {
        return;
    }

    kinfo("task %s %#018lx exit......", current_task->name, current_task);

    current_task->state = TASK_DIED;

    task_t *next = task_search(TASK_READY, current_cpu_id);

    task_switch_to(NULL, NULL, next);
}
