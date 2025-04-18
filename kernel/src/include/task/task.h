#pragma once

#include <klibc.h>
#include <acpi/acpi.h>
#include <task/fsgsbase.h>
#include <mm/page.h>
#include <task/signal.h>

#define STACK_SIZE 32768UL

#define USER_STACK_TOP USER_BRK_START
#define USER_STACK_SIZE (1 * 1024 * 1024)

#define IOBITMAP_SIZE (65536 / 8)

typedef struct tss
{
    uint32_t reserved0;
    uint64_t rsp0;
    uint64_t rsp1;
    uint64_t rsp2;
    uint64_t reserved1;
    uint64_t ist1;
    uint64_t ist2;
    uint64_t ist3;
    uint64_t ist4;
    uint64_t ist5;
    uint64_t ist6;
    uint64_t ist7;
    uint64_t reserved2;
    uint16_t reserved3;
    uint16_t iomapbaseaddr;
    uint8_t iomap[IOBITMAP_SIZE];
} __attribute__((packed)) tss_t;

typedef struct
{
    uint16_t fcw;
    uint16_t fsw;
    uint16_t ftw;
    uint16_t fop;
    uint64_t word2;
    uint64_t word3;
    uint32_t mxscr;
    uint32_t mxcsr_mask;
    uint64_t mm[16];
    uint64_t xmm[32];
    uint64_t rest[12];
} __attribute__((aligned(16))) fpu_context;

typedef enum task_state
{
    TASK_RUNNING = 1,
    TASK_READY,
    TASK_BLOCKING,
    TASK_DIED,
} task_state_t;

#define TASK_NAME_LEN 64

typedef struct task_thread
{
    uint64_t elf_mapping_start;
    uint64_t elf_mapping_end;

    uint64_t fs;
    uint64_t gs;

    uint64_t fsbase;
    uint64_t gsbase;
} task_thread_t;

#define MAX_FD_NUM 16

struct scheme;
typedef struct scheme scheme_t;

struct fsd;
typedef struct fsd fsd_t;

typedef struct task
{
    uint64_t self_ref;
    char name[TASK_NAME_LEN];
    uint64_t task_id;
    uint64_t parent_task_id;
    uint64_t waitpid;
    uint64_t syscall_stack;
    uint64_t uid;
    uint64_t on_cpu;
    uint64_t brk_start;
    uint64_t brk_end;
    struct List list;
    struct pt_regs *context;
    uint64_t jiffies;
    page_directory_t *pgdir;
    task_thread_t *thread;
    task_state_t state;
    fpu_context *fpu;
    fsd_t *fsds[MAX_FD_NUM];
    scheme_t *schemes[MAX_FD_NUM];
    int status;
    sigaction_t actions[MAXSIG];
    uint64_t signal;
    uint64_t blocked;
    bool need_schedule;
    bool userspace_io_allowed;
} task_t;

#define KERNEL_USER 0
#define NORMAL_USER 1

#define MAX_TASK_NUM 2048

extern bool can_schedule;

extern task_t *tasks[MAX_TASK_NUM];
extern task_t *idle_tasks[MAX_CPU_NUM];

uint32_t get_cpuid_by_lapic_id(uint32_t lapic_id);

#define current_cpu_id get_cpuid_by_lapic_id(lapic_id())

void timer_handler(uint8_t irq, uint64_t param, struct pt_regs *regs);

task_t *task_create(const char *name, void (*entry)(), uint64_t uid);
void task_switch_to(struct pt_regs *curr, task_t *prev, task_t *next);

static inline task_t *get_current_task()
{
    task_t *ret = (task_t *)read_kgsbase();
    // __asm__ __volatile__("movq %%gs:%%rax, %%rax" : "=a"(ret) : "a"(offsetof(task_t, self_ref)));
    return ret;
}

#define current_task get_current_task()

task_t *task_search(task_state_t state, uint32_t cpu_id);
task_t *get_task(uint64_t pid);

int task_block(task_t *task, struct List *blist, task_state_t state, int timeout_ms);
void task_unblock(task_t *task, int reason);

void task_to_user_mode(uint64_t entry, uint64_t load_start, uint64_t load_end, char **argv, char **envp);

void task_exit(int code);

uint64_t sys_fork(struct pt_regs *regs);
uint64_t sys_waitpid(uint64_t pid, int *status);
void sys_iopl(uint64_t level);
void sys_yield();

void tss_init();

void task_init();
