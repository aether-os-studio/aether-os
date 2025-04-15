#include <libsyscall.h>
#include <libdaemon.h>
#include <stdio.h>
#include "partition.h"

uint64_t fsd_daemon(daemon_t *daemon)
{
    printf("fs daemon is running\n");

    partition_init();

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
