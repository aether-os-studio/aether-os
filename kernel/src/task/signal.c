#include <task/task.h>
#include <task/signal.h>
#include <kprint.h>

typedef struct signal_frame
{
    uint64_t restorer;
    uint64_t sig;
    uint64_t blocked;

    uint64_t r15;
    uint64_t r14;
    uint64_t r13;
    uint64_t r12;
    uint64_t r11;
    uint64_t r10;
    uint64_t r9;
    uint64_t r8;
    uint64_t rbx;
    uint64_t rcx;
    uint64_t rdx;
    uint64_t rsi;
    uint64_t rdi;
    uint64_t rbp;
    uint64_t ds;
    uint64_t es;
    uint64_t rax;
    uint64_t rip;
} signal_frame_t;

// 获取信号屏蔽位图
int sys_sgetmask()
{
    return current_task->blocked;
}

// 设置信号屏蔽位图
int sys_ssetmask(int newmask)
{
    if (newmask == 0)
    {
        return -1;
    }

    int old = current_task->blocked;
    current_task->blocked = newmask & ~SIGMASK(SIGKILL);

    return old;
}

int sys_signal(int sig, uint64_t handler, uint64_t restorer)
{
    if (sig < MINSIG || sig > MAXSIG || sig == SIGKILL)
    {
        return 0;
    }

    sigaction_t *ptr = &current_task->actions[sig - 1];
    ptr->mask = 0;
    ptr->handler = (void (*)(int))handler;
    ptr->flags = SIG_ONESHOT | SIG_NOMASK;
    ptr->restorer = (void (*)(void))restorer;
    return handler;
}

int sys_sigaction(int sig, sigaction_t *action, sigaction_t *oldaction)
{
    if (sig < MINSIG || sig > MAXSIG || sig == SIGKILL)
    {
        return 0;
    }

    sigaction_t *ptr = &current_task->actions[sig - 1];
    if (oldaction)
    {
        *oldaction = *ptr;
    }

    *ptr = *action;

    if (ptr->flags & SIG_NOMASK)
    {
        ptr->mask = 0;
    }
    else
    {
        ptr->mask |= SIGMASK(sig);
    }

    return 0;
}

int sys_kill(int pid, int sig)
{
    if (sig < MINSIG || sig > MAXSIG)
    {
        return 0;
    }

    task_t *task = get_task(pid);

    if (!task)
    {
        return 0;
    }

    if (task->uid == KERNEL_USER)
    {
        return 0;
    }

    kinfo("kill task %s pid %d signal %d", task->name, pid, sig);

    task->signal |= SIGMASK(sig);

    if (task->state == TASK_BLOCKING)
    {
        task_unblock(task, -EINTR);
    }

    return 0;
}

void sys_sendsignal(uint64_t pid, int sig)
{
    task_t *task = get_task(pid);
    task->signal |= SIGMASK(sig);
}

void task_signal()
{
    cli();

    uint64_t map = current_task->signal & (~current_task->blocked);
    if (!map)
        return;

    int sig = 1;
    for (; sig <= MAXSIG; sig++)
    {
        if (map & SIGMASK(sig))
        {
            current_task->signal &= (~SIGMASK(sig));
            break;
        }
    }

    sigaction_t *ptr = &current_task->actions[sig - 1];

    if (ptr->handler == SIG_IGN)
    {
        return;
    }

    if (ptr->handler == SIG_DFL && sig == SIGCHLD)
    {
        return;
    }

    if (ptr->handler == SIG_DFL)
    {
        task_exit(SIGMASK(sig));
    }

    struct pt_regs *iframe = current_task->context;

    signal_frame_t *frame = (signal_frame_t *)iframe->rsp - 1;

    frame->rax = iframe->rax;

    frame->ds = iframe->ds;
    frame->es = iframe->es;

    frame->rbp = iframe->rbp;
    frame->rdi = iframe->rdi;
    frame->rsi = iframe->rsi;

    frame->rbx = iframe->rbx;
    frame->rcx = iframe->rcx;
    frame->rdx = iframe->rdx;

    frame->r8 = iframe->r8;
    frame->r9 = iframe->r9;
    frame->r10 = iframe->r10;
    frame->r11 = iframe->r11;
    frame->r12 = iframe->r12;
    frame->r13 = iframe->r13;
    frame->r14 = iframe->r14;
    frame->r15 = iframe->r15;

    frame->rip = iframe->rip;

    frame->blocked = 0;

    if (ptr->flags & SIG_NOMASK)
    {
        frame->blocked = current_task->blocked;
    }

    frame->sig = sig;
    frame->restorer = (uint64_t)ptr->restorer;

    iframe->rsp = (uint64_t)frame;
    iframe->rip = (uint64_t)ptr->handler;

    if (ptr->flags & SIG_ONESHOT)
    {
        ptr->handler = SIG_DFL;
    }

    current_task->blocked |= ptr->mask;
}
