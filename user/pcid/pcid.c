#include <libdaemon.h>
#include <stdio.h>
#include <stdlib.h>

user_scheme_t pcid_scheme;

uint64_t pcid_daemon(daemon_t *daemon)
{
    printf("pcid daemon is running\n");

    memset(&pcid_daemon, 0, sizeof(user_scheme_t));

    scheme_create("/scheme/pcid", &pcid_scheme);

    finish_daemon(daemon);

    int fd = open("/scheme/acpid", 0, 0);
    if (fd < 0)
    {
        printf("open acpid scheme failed\n");
        return -1;
    }

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}

int main()
{
    return start_daemon(pcid_daemon);
}
