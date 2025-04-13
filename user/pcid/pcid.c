#include <libdaemon.h>
#include <stdio.h>
#include <stdlib.h>

user_scheme_t pcid_scheme;

uint64_t pcid_daemon(daemon_t *daemon)
{
    printf("pcid daemon is running\n");

    init_scheme(&pcid_scheme);

    scheme_create("/scheme/pcid", &pcid_scheme);

    finish_daemon(daemon);

    int fd = open("/scheme/acpid/MCFG", 0, 0);
    if (fd < 0)
    {
        printf("open acpid scheme failed\n");
        return -1;
    }

    close(fd);

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}

int main()
{
    return start_daemon(pcid_daemon);
}
