#pragma once

#include <klibc.h>

#define SYS_READ 0
#define SYS_WRITE 1
#define SYS_OPEN 2
#define SYS_CLOSE 3

#define SYS_EXIT 60

extern void syscall_exception();

void syscall_init();
