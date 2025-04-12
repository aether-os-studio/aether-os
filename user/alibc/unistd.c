#include <libsyscall.h>

int read(int fd, void *buf, int len)
{
    enter_syscall(SYS_READ, (uint64_t)fd, (uint64_t)buf, (uint64_t)len, 0, 0, 0);
}

int write(int fd, void *buf, int len)
{
    enter_syscall(SYS_WRITE, (uint64_t)fd, (uint64_t)buf, (uint64_t)len, 0, 0, 0);
}
