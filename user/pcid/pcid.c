#include <libdaemon.h>
#include <stdio.h>
#include <stdlib.h>
#include "pci.h"

bool pcie;

user_scheme_t pcid_scheme;

uint64_t pcid_daemon(daemon_t *daemon)
{
    printf("pcid daemon is running\n");

    init_scheme(&pcid_scheme);

    scheme_create("/scheme/pcid", &pcid_scheme);

    pcie = true;

    int fd = open("/scheme/acpid/MCFG", 0, 0);
    if (fd < 0)
    {
        printf("open acpid scheme failed\n");
        pcie = false;
    }

    int len = ioctl(fd, SCHEME_IOCTL_GETSIZE, 0);
    if (len < 0)
    {
        printf("ioctl failed\n");
        close(fd);
        pcie = false;
    }

    MCFG *buf = (MCFG *)malloc(len);
    if (!buf)
    {
        printf("malloc failed\n");
        close(fd);
        pcie = false;
    }
    memset(buf, 0, len);

    int ret = read(fd, buf, len);
    if (ret != len)
    {
        printf("read failed\n");
        free(buf);
        close(fd);
        pcie = false;
    }

    if (pcie)
    {
        close(fd);
    }

    iopl(3);

    init_pci(buf, pcie);

    free(buf);

    finish_daemon(daemon);

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}

int main()
{
    return start_daemon(pcid_daemon);
}
