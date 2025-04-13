#include <libdaemon.h>
#include <stdio.h>
#include <stdlib.h>
#include "mcfg.h"

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

    int len = ioctl(fd, SCHEME_IOCTL_GETSIZE, 0);
    if (len < 0)
    {
        printf("ioctl failed\n");
        close(fd);
        return -1;
    }

    MCFG *buf = (MCFG *)malloc(len);
    memset(buf, 0, len);

    read(fd, buf, len);

    printf("MCFG Signature = %s", buf->Header.Signature);

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
