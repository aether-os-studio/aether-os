#include <libsyscall.h>

__attribute__((naked)) uint64_t enter_syscall(uint64_t idx, uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5, uint64_t arg6)
{
    __asm__ __volatile__(
        "movq %0, %%rdi\n\t"
        "movq %1, %%rsi\n\t"
        "movq %2, %%rdx\n\t"
        "movq %3, %%r10\n\t"
        "movq %4, %%r8\n\t"
        "movq %5, %%r9\n\t"
        "syscall\n\t"
        "ret\n\t" ::"r"(arg1),
        "r"(arg2), "r"(arg3), "r"(arg4), "r"(arg5), "r"(arg6), "a"(idx));
}

int signal(int sig, uint64_t handler)
{
    return enter_syscall(SYS_SIGNAL, (uint64_t)sig, handler, (uint64_t)&restorer, 0, 0, 0);
}

int sigaction(int sg, sigaction_t *act, sigaction_t *oldact)
{
    return enter_syscall(SYS_SIGACTION, (uint64_t)sg, (uint64_t)act, (uint64_t)oldact, 0, 0, 0);
}

void ssetmask(uint64_t mask)
{
    enter_syscall(SYS_SETMASK, mask, 0, 0, 0, 0, 0);
}

void send_signal(int pid, int sig)
{
    enter_syscall(SYS_SENDSIGNAL, pid, sig, 0, 0, 0, 0);
}

int getpid()
{
    return enter_syscall(SYS_GETPID, 0, 0, 0, 0, 0, 0);
}

int read(int fd, void *buf, int len)
{
    return enter_syscall(SYS_READ, (uint64_t)fd, (uint64_t)buf, (uint64_t)len, 0, 0, 0);
}

int write(int fd, void *buf, int len)
{
    return enter_syscall(SYS_WRITE, (uint64_t)fd, (uint64_t)buf, (uint64_t)len, 0, 0, 0);
}

int fork()
{
    return (int)enter_syscall(SYS_FORK, 0, 0, 0, 0, 0, 0);
}

void iopl(uint64_t level)
{
    enter_syscall(SYS_IOPL, level, 0, 0, 0, 0, 0);
}
