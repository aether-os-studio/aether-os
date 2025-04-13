#pragma once

#include <libsyscall.h>

typedef struct daemon
{
    int fd;
    int flags;
} daemon_t;

uint64_t start_daemon(uint64_t (*func)(daemon_t *));
void finish_daemon(daemon_t *daemon);
