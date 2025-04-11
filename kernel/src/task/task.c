#include <task/task.h>
#include <mm/hhdm.h>
#include <mm/frame.h>
#include <mm/page.h>
#include <irq/gate.h>
#include <irq/irq.h>
#include <kprint.h>

task_t *tasks[MAX_TASK_NUM];
task_t *idle_task;

extern uint64_t cpu_count;

uint32_t alloc_cpuid()
{
    static uint32_t i = 0;
    return i++;
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

static task_t *task_search(task_state_t state)
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

        if (task == NULL || ptr->jiffies < task->jiffies)
            task = ptr;
    }

    if (task == NULL && state == TASK_READY)
    {
        task = idle_task;
    }

    return task;
}

void timer_handler(uint8_t irq, uint64_t param, struct pt_regs *regs)
{
    current_task->jiffies++;

    task_t *next = task_search(TASK_READY);
    if (next->on_cpu != current_cpu_id || next == current_task)
    {
        return;
    }
    task_switch_to(regs, current_task, next);
}

task_t *task_create(const char *name, void (*entry)())
{
    task_t *task = get_free_task();

    uint64_t stack = (uint64_t)task + PAGE_SIZE;

    stack -= sizeof(struct pt_regs);
    struct pt_regs *task_frame = (struct pt_regs *)stack;

    task_frame->rflags = 0x200;
    task_frame->rsp = stack;
    task_frame->rip = (uint64_t)entry;
    task_frame->cs = SELECTOR_KERNEL_CS;
    task_frame->ss = SELECTOR_KERNEL_DS;
    task_frame->ds = SELECTOR_KERNEL_DS;
    task_frame->es = SELECTOR_KERNEL_DS;
    task->context = task_frame;

    task->on_cpu = alloc_cpuid();
    task->pgdir = clone_directory(get_kernel_page_dir());
    task->state = TASK_READY;
    task->magic = AETHER_MAGIC;

    return task;
}

void idle_thread()
{
    kdebug("idle thread is running");

    while (1)
    {
        sti();
        hlt();
    }
}

void init_thread()
{
    kdebug("init thread is running");

    while (1)
    {
        sti();
        hlt();
    }
}

bool task_initialized = false;

void task_init()
{
    memset(tasks, 0, sizeof(tasks));
    idle_task = task_create("idle", idle_thread);
    task_create("init", init_thread);

    irq_register(APIC_TIMER_INTERRUPT_VECTOR, timer_handler, 0, NULL, "APIC TIMER");

    write_gsbase((uint64_t)idle_task);
    task_initialized = true;
    task_switch_to(NULL, NULL, idle_task);
}

void task_switch_to(struct pt_regs *curr, task_t *prev, task_t *next)
{
    if (curr != NULL && prev != NULL)
    {
        memcpy(prev->context, curr, sizeof(struct pt_regs));
    }

    if (next->magic == AETHER_MAGIC)
    {
        // Start to schedule
        write_gsbase((uint64_t)next);
        __asm__ __volatile__("movq %0, %%rsp\n\t"
                             "jmp ret_from_intr" ::"r"(next->context));
    }
}
