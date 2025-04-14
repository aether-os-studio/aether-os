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

int getppid()
{
    return enter_syscall(SYS_GETPPID, 0, 0, 0, 0, 0, 0);
}

int read(int fd, void *buf, int len)
{
    return enter_syscall(SYS_READ, (uint64_t)fd, (uint64_t)buf, (uint64_t)len, 0, 0, 0);
}

int write(int fd, void *buf, int len)
{
    return enter_syscall(SYS_WRITE, (uint64_t)fd, (uint64_t)buf, (uint64_t)len, 0, 0, 0);
}

int open(const char *name, int mode, int flags)
{
    return enter_syscall(SYS_OPEN, (uint64_t)name, mode, flags, 0, 0, 0);
}

int close(int fd)
{
    return enter_syscall(SYS_CLOSE, (uint64_t)fd, 0, 0, 0, 0, 0);
}

int ioctl(int fd, int cmd, int arg)
{
    return enter_syscall(SYS_IOCTL, fd, cmd, arg, 0, 0, 0);
}

int fork()
{
    return (int)enter_syscall(SYS_FORK, 0, 0, 0, 0, 0, 0);
}

void iopl(uint64_t level)
{
    enter_syscall(SYS_IOPL, level, 0, 0, 0, 0, 0);
}

int waitpid(int pid, int *status)
{
    enter_syscall(SYS_WAITPID, (uint64_t)pid, (uint64_t)status, 0, 0, 0, 0);
}

void load_module(const char *name)
{
    enter_syscall(SYS_LOAD_MODULE, (uint64_t)name, 0, 0, 0, 0, 0);
}

void get_bootstrap_info(bootstrap_info_t *info)
{
    enter_syscall(SYS_GET_INFO, (uint64_t)info, 0, 0, 0, 0, 0);
}

uint64_t physmap(uint64_t addr, uint64_t size, uint64_t flags)
{
    return enter_syscall(SYS_PHYSMAP, addr, size, flags, 0, 0, 0);
}

void scheme_create(const char *name, user_scheme_t *addr)
{
    enter_syscall(SYS_SCHEME_CREATE, (uint64_t)name, (uint64_t)addr, 0, 0, 0, 0);
}

uint64_t virttophys(uint64_t virt)
{
    return enter_syscall(SYS_VIRTTOPHYS, virt, 0, 0, 0, 0, 0);
}

uint64_t alloc_dma(uint64_t count)
{
    return enter_syscall(SYS_DMA_ALLOC, count, 0, 0, 0, 0, 0);
}

void free_dma(uint64_t addr, uint64_t count)
{
    return enter_syscall(SYS_DMA_FREE, addr, count, 0, 0, 0, 0);
}
