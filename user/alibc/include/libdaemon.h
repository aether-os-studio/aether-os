#pragma once

#include <libsyscall.h>

#define wait_until(cond) \
    ({                   \
        while (!(cond))  \
            ;            \
    })

#define wait_until_expire(cond, max)          \
    ({                                        \
        uint64_t __wcounter__ = (max);        \
        while (!(cond) && __wcounter__-- > 1) \
            ;                                 \
        __wcounter__;                         \
    })

typedef struct daemon
{
    int fd;
    int flags;
} daemon_t;

uint64_t start_daemon(uint64_t (*func)(daemon_t *));
void finish_daemon(daemon_t *daemon);
