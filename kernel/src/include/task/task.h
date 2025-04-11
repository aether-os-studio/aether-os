#pragma once

#include <klibc.h>
#include <acpi/acpi.h>
#include <task/fsgsbase.h>

#define AETHER_MAGIC 0x1234567887654321

#define MAX_CPU_NUM 256

typedef enum task_state
{
    TASK_RUNNING = 1,
    TASK_READY,
    TASK_BLOCKING,
    TASK_DIED,
} task_state_t;

typedef struct task
{
    uint64_t task_id;
    uint64_t on_cpu;
    struct List list;
    struct pt_regs *context;
    uint64_t jiffies;
    uint64_t kernel_stack;
    task_state_t state;
    uint64_t magic;
} task_t;

#define MAX_TASK_NUM 1024

extern task_t *tasks[MAX_TASK_NUM];
extern task_t *idle_task;

uint32_t get_cpuid_by_lapic_id(uint32_t lapic_id);

#define current_cpu_id get_cpuid_by_lapic_id(lapic_id())

task_t *task_create(const char *name, void (*entry)());
void task_switch_to(struct pt_regs *curr, task_t *prev, task_t *next);

#define current_task ((task_t *)read_gsbase())

void task_init();
