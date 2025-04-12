#include <libsyscall.h>

void exit(int code)
{
    enter_syscall(SYS_EXIT, code, 0, 0, 0, 0, 0);
}
