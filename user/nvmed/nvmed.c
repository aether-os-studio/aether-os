#include <libsyscall.h>
#include <libdaemon.h>
#include <stdio.h>
#include "pci.h"
#include "nvme.h"

NVME_NAMESPACE *namespaces[MAX_NVME_DEV_NUM];
uint64_t nvme_device_num = 0;

uint64_t nvmed_read(uint64_t buf, uint64_t len, uint64_t offset, char *target_name)
{
    if (is_digit(target_name[0]))
    {
        uint8_t idx = 0;

        idx = target_name[0] - '0';

        if (strlen(target_name) == 2 && is_digit(target_name[1]))
        {
            idx += (target_name[1] - '0') * 10;
        }

        if (idx > MAX_NVME_DEV_NUM)
        {
            return (uint64_t)-EINVAL;
        }

        NVME_NAMESPACE *ns = namespaces[idx];

        return (uint64_t)NVMETransfer(ns, (void *)buf, offset / 512, len / 512, false);
    }

    return 0;
}

uint64_t nvmed_write(uint64_t buf, uint64_t len, uint64_t offset, char *target_name)
{
    if (is_digit(target_name[0]))
    {
        uint8_t idx = 0;

        idx = target_name[0] - '0';

        if (strlen(target_name) == 2 && is_digit(target_name[1]))
        {
            idx += (target_name[1] - '0') * 10;
        }

        if (idx > MAX_NVME_DEV_NUM)
        {
            return (uint64_t)-EINVAL;
        }

        NVME_NAMESPACE *ns = namespaces[idx];

        return (uint64_t)NVMETransfer(ns, (void *)buf, offset / 512, len / 512, true);
    }

    return 0;
}

uint64_t nvmed_ioctl(uint64_t cmd, uint64_t arg, uint64_t offset, char *target_name)
{
    switch (cmd)
    {
    case SCHEME_IOCTL_GETBLKSIZE:
        return 512;
    default:
        break;
    }

    return 0;
}

user_scheme_t nvmed_scheme;

uint64_t nvmed_daemon(daemon_t *daemon)
{
    printf("nvme daemon is running\n");

    int fd = open("/scheme/pcid/nvme", 0, 0);
    if (fd < 0)
    {
        printf("open pcid scheme failed\n");
        return -1;
    }

    pci_device_t nvme_pci_device;
    memset(&nvme_pci_device, 0, sizeof(pci_device_t));
    int ret = read(fd, &nvme_pci_device, sizeof(pci_device_t));
    if (ret != sizeof(pci_device_t))
    {
        printf("read nvme device failed\n");
        close(fd);
        return -1;
    }

    nvme_driver_init(nvme_pci_device.bars[0].address, nvme_pci_device.bars[0].size);

    close(fd);

    init_scheme(&nvmed_scheme);

    scheme_create("/scheme/nvmed", &nvmed_scheme);

    finish_daemon(daemon);

    while (1)
    {
        if (nvmed_scheme.command.cmd != 0)
        {
            uint64_t result = 0;
            switch (nvmed_scheme.command.cmd)
            {
            case SCHEME_COMMAND_READ:
                result = nvmed_read(nvmed_scheme.command.a, nvmed_scheme.command.b, nvmed_scheme.command.c, (char *)nvmed_scheme.command.d);
                break;
            case SCHEME_COMMAND_WRITE:
                result = nvmed_write(nvmed_scheme.command.a, nvmed_scheme.command.b, nvmed_scheme.command.c, (char *)nvmed_scheme.command.d);
                break;
            case SCHEME_COMMAND_IOCTL:
                result = nvmed_ioctl(nvmed_scheme.command.a, nvmed_scheme.command.b, nvmed_scheme.command.c, (char *)nvmed_scheme.command.d);
                break;
            default:
                printf("unknown command\n");
                break;
            }

            nvmed_scheme.command.a = result;
            nvmed_scheme.command.cmd = 0;
        }

        __asm__ __volatile__("pause");
    }
}

int main()
{
    return start_daemon(nvmed_daemon);
}
