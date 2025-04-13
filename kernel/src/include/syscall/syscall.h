#pragma once

#include <klibc.h>

enum
{
    SYS_READ = 1,
    SYS_WRITE,
    SYS_OPEN,
    SYS_CLOSE,
    SYS_SIGACTION,
    SYS_SIGNAL,
    SYS_SETMASK,
    SYS_SENDSIGNAL,
    SYS_EXIT,
    SYS_GETPID,
    SYS_FORK,
    SYS_WAITPID,
    SYS_LOAD_MODULE,
    SYS_IOPL,

    SYS_NUM,
};

extern void syscall_exception();

void syscall_init();
