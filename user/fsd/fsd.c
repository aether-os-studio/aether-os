#include <libsyscall.h>
#include <libdaemon.h>
#include <stdio.h>
#include "cache.h"

uint64_t fsd_daemon(daemon_t *daemon)
{
    printf("fsd daemon is running\n");

    cache_init();

    finish_daemon(daemon);

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}

int main()
{
    start_daemon(fsd_daemon);
}
