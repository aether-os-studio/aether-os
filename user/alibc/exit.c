#include <libsyscall.h>

void exit(int code)
{
    enter_syscall(SYS_EXIT, code, 0, 0, 0, 0, 0);
}

void abort()
{
    enter_syscall(SYS_EXIT, (uint64_t)-1, 0, 0, 0, 0, 0);
}
