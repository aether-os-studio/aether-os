#include <syscall/syscall.h>
#include <kprint.h>
#include <task/task.h>
#include <irq/irq.h>
#include <irq/gate.h>
#include <task/signal.h>

uint64_t sys_write(uint64_t fd, uint64_t buf, uint64_t len)
{
    if (fd == 1)
    {
        printk("%s", (char *)buf);
    }
    else if (fd == 2)
    {
        printk_color(RED, BLACK, "%s", (char *)buf);
    }
}

uint64_t sys_exit(uint64_t code)
{
    task_exit((int)code);

    return 0xFFFFFFFFFFFFFFFF;
}

extern void sys_load_module(const char *);

void *real_memcpy(void *dst, void *src, long len)
{
    return memcpy(dst, src, len);
}

uint64_t switch_to_kernel_stack()
{
    return current_task->syscall_stack;
}

void syscall_handler(struct pt_regs *regs, struct pt_regs *user_regs)
{
    regs->rip = regs->rcx;
    regs->rflags = regs->r11;
    regs->cs = SELECTOR_USER_CS;
    regs->ss = SELECTOR_USER_DS;
    regs->rsp = (uint64_t)(user_regs + 1);

    uint64_t idx = regs->rax;

    uint64_t arg1 = regs->rdi;
    uint64_t arg2 = regs->rsi;
    uint64_t arg3 = regs->rdx;
    uint64_t arg4 = regs->r10;
    uint64_t arg5 = regs->r8;
    uint64_t arg6 = regs->r9;

    switch (idx)
    {
    case SYS_WRITE:
        regs->rax = sys_write(arg1, arg2, arg3);
        break;

    case SYS_EXIT:
        regs->rax = sys_exit(arg1);
        break;
    case SYS_GETPID:
        regs->rax = current_task->task_id;
        break;
    case SYS_GETPPID:
        regs->rax = current_task->parent_task_id;
        break;
    case SYS_FORK:
        regs->rax = sys_fork(regs);
        break;
    case SYS_WAITPID:
        regs->rax = sys_waitpid(arg1, (int *)arg2);
        break;

    case SYS_SIGNAL:
        regs->rax = sys_signal(arg1, arg2, arg3);
        break;
    case SYS_SIGACTION:
        regs->rax = sys_sigaction(arg1, (sigaction_t *)arg2, (sigaction_t *)arg3);
        break;
    case SYS_SETMASK:
        regs->rax = sys_ssetmask(arg1);
        break;
    case SYS_SENDSIGNAL:
        sys_sendsignal(arg1, arg2);
        regs->rax = 0;
        break;

    case SYS_LOAD_MODULE:
        sys_load_module((const char *)arg1);
        regs->rax = 0;
        break;

    case SYS_IOPL:
        sys_iopl(arg1);
        regs->rax = 0;
        break;

    default:
        regs->rax = ((uint64_t)-ENOSYS);
        break;
    }
}

// MSR寄存器地址定义
#define MSR_EFER 0xC0000080         // EFER MSR寄存器
#define MSR_STAR 0xC0000081         // STAR MSR寄存器
#define MSR_LSTAR 0xC0000082        // LSTAR MSR寄存器
#define MSR_SYSCALL_MASK 0xC0000084 // SYSCALL_MASK MSR寄存器

void syscall_init()
{
    uint64_t efer;

    // 1. 启用 EFER.SCE (System Call Extensions)
    efer = rdmsr(MSR_EFER);
    efer |= 1; // 设置 SCE 位
    wrmsr(MSR_EFER, efer);

    uint16_t cs_sysret_cmp = SELECTOR_USER_CS - 16;
    uint16_t ss_sysret_cmp = SELECTOR_USER_DS - 8;
    uint16_t cs_syscall_cmp = SELECTOR_KERNEL_CS;
    uint16_t ss_syscall_cmp = SELECTOR_KERNEL_DS - 8;

    if (cs_sysret_cmp != ss_sysret_cmp)
    {
        kerror("Sysret offset is not valid (1)\n");
        return;
    }

    if (cs_syscall_cmp != ss_syscall_cmp)
    {
        kerror("Syscall offset is not valid (2)\n");
        return;
    }

    // 2. 设置 STAR MSR
    uint64_t star = 0;
    star = ((uint64_t)(SELECTOR_USER_DS - 8) << 48) | // SYSRET 的基础 CS
           ((uint64_t)SELECTOR_KERNEL_CS << 32);      // SYSCALL 的 CS
    wrmsr(MSR_STAR, star);

    // 3. 设置 LSTAR MSR (系统调用入口点)
    wrmsr(MSR_LSTAR, (uint64_t)syscall_exception);

    // 4. 设置 SYSCALL_MASK MSR (RFLAGS 掩码)
    // wrmsr(MSR_SYSCALL_MASK, (1 << 9));
}
