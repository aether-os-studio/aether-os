#include <libdaemon.h>
#include <stdio.h>
#include <stdlib.h>
#include "pci.h"

uint64_t pcid_read(uint64_t buf, uint64_t len, uint64_t offset, char *target_name)
{
    if (!strncmp(target_name, "ahci", 4))
    {
        pci_device_t *device = pci_find_class(0x010601);
        if (device)
        {
            memcpy((pci_device_t *)buf, device, sizeof(pci_device_t));
            return sizeof(pci_device_t);
        }
    }

    if (!strncmp(target_name, "nvme", 4))
    {
        pci_device_t *device = pci_find_class(0x010802);
        if (device)
        {
            memcpy((pci_device_t *)buf, device, sizeof(pci_device_t));
            return sizeof(pci_device_t);
        }
    }

    if (!strncmp(target_name, "xhci", 4))
    {
        pci_device_t *device = pci_find_class(0x0C0330);
        if (device)
        {
            memcpy((pci_device_t *)buf, device, sizeof(pci_device_t));
            return sizeof(pci_device_t);
        }
    }

    return 0;
}

uint64_t pcid_write(uint64_t buf, uint64_t len, uint64_t offset, char *target_name)
{
    return (uint64_t)-1;
}

uint64_t pcid_ioctl(uint64_t cmd, uint64_t arg, uint64_t offset, char *target_name)
{
    switch (cmd)
    {
    case SCHEME_IOCTL_GETSIZE:
    default:
        break;
    }

    return 0;
}

bool pcie;

user_scheme_t pcid_scheme;

uint64_t pcid_daemon(daemon_t *daemon)
{
    printf("pci daemon is running\n");

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
        printf("malloc MCFG buffer failed\n");
        close(fd);
        pcie = false;
    }
    memset(buf, 0, len);

    int ret = read(fd, buf, len);
    if (ret != len)
    {
        printf("read MCFG failed\n");
        free(buf);
        close(fd);
        pcie = false;
    }

    if (pcie)
    {
        close(fd);
    }

    init_pci(buf, pcie);

    free(buf);

    finish_daemon(daemon);

    while (1)
    {
        if (pcid_scheme.command.cmd != 0)
        {
            uint64_t result = 0;
            switch (pcid_scheme.command.cmd)
            {
            case SCHEME_COMMAND_READ:
                result = pcid_read(pcid_scheme.command.a, pcid_scheme.command.b, pcid_scheme.command.c, (char *)pcid_scheme.command.d);
                break;
            case SCHEME_COMMAND_WRITE:
                result = pcid_write(pcid_scheme.command.a, pcid_scheme.command.b, pcid_scheme.command.c, (char *)pcid_scheme.command.d);
                break;
            case SCHEME_COMMAND_IOCTL:
                result = pcid_ioctl(pcid_scheme.command.a, pcid_scheme.command.b, pcid_scheme.command.c, (char *)pcid_scheme.command.d);
                break;
            default:
                printf("unknown command\n");
                break;
            }

            pcid_scheme.command.a = result;
            pcid_scheme.command.cmd = 0;
        }

        __asm__ __volatile__("pause");
    }
}

int main()
{
    return start_daemon(pcid_daemon);
}
