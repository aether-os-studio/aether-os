#include <stdio.h>
#include "ahci.h"

user_scheme_t ahcid_scheme;

uint64_t ahcid_read(uint64_t buf, uint64_t len, uint64_t offset, char *target_name)
{
    if (is_digit(target_name[0]))
    {
        uint8_t idx = 0;

        idx = target_name[0] - '0';

        if (strlen(target_name) == 2 && is_digit(target_name[1]))
        {
            idx += (target_name[1] - '0') * 10;
        }

        struct hba_device *device = drv->hba.ports[idx]->device;

        if (drv->hba.ports[idx] == NULL)
        {
            return 0;
        }

        struct blkio_req req;
        memset(&req, 0, sizeof(struct blkio_req));

        req.buf = (uint64_t)op_buffer;
        req.lba = offset / device->block_size;
        req.len = len;

        device->ops.submit(device, &req);

        memcpy((void *)buf, op_buffer, len);

        return len;
    }

    return 0;
}

uint64_t ahcid_write(uint64_t buf, uint64_t len, uint64_t offset, char *target_name)
{
    if (is_digit(target_name[0]))
    {
        uint8_t idx = 0;

        idx = target_name[0] - '0';

        if (strlen(target_name) == 2 && is_digit(target_name[1]))
        {
            idx += (target_name[1] - '0') * 10;
        }

        if (drv->hba.ports[idx] == NULL)
        {
            return 0;
        }

        struct hba_device *device = drv->hba.ports[idx]->device;

        struct blkio_req req;
        memset(&req, 0, sizeof(struct blkio_req));

        req.flags |= BLKIO_WRITE;

        memcpy(op_buffer, (void *)buf, len);

        req.buf = (uint64_t)op_buffer;
        req.lba = offset / device->block_size;
        req.len = len;

        device->ops.submit(device, &req);

        return len;
    }

    return 0;
}

uint64_t ahcid_ioctl(uint64_t cmd, uint64_t arg, uint64_t offset, char *target_name)
{
    switch (cmd)
    {
    case SCHEME_IOCTL_GETBLKSIZE:
    {

        if (is_digit(target_name[0]))
        {
            uint8_t idx = 0;

            idx = target_name[0] - '0';

            if (strlen(target_name) == 2 && is_digit(target_name[1]))
            {
                idx += (target_name[1] - '0') * 10;
            }

            if (drv->hba.ports[idx] == NULL)
            {
                return 0;
            }

            struct hba_device *device = drv->hba.ports[idx]->device;

            return device->block_size;
        }
    }

    default:
        break;
    }

    return 0;
}

uint64_t main_loop()
{
    for (uint32_t port = 0; port < drv->hba.ports_num; port++)
    {
        if (drv->hba.ports[port] == NULL)
            continue;

        char buf[30];
        sprintf(buf, "/scheme/block//scheme/ahcid/%d", port);

        int fd = open(buf, 0, 0);

        ioctl(fd, SCHEME_IOCTL_REGIST_BLKDEV, drv->hba.ports[port]->device->block_size);

        close(fd);
    }

    while (1)
    {
        if (ahcid_scheme.command.cmd != 0)
        {
            uint64_t result = 0;
            switch (ahcid_scheme.command.cmd)
            {
            case SCHEME_COMMAND_READ:
                result = ahcid_read(ahcid_scheme.command.a, ahcid_scheme.command.b, ahcid_scheme.command.c, (char *)ahcid_scheme.command.d);
                break;
            case SCHEME_COMMAND_WRITE:
                result = ahcid_write(ahcid_scheme.command.a, ahcid_scheme.command.b, ahcid_scheme.command.c, (char *)ahcid_scheme.command.d);
                break;
            case SCHEME_COMMAND_IOCTL:
                result = ahcid_ioctl(ahcid_scheme.command.a, ahcid_scheme.command.b, ahcid_scheme.command.c, (char *)ahcid_scheme.command.d);
                break;
            default:
                printf("unknown command\n");
                break;
            }

            ahcid_scheme.command.a = result;
            ahcid_scheme.command.cmd = 0;
        }

        __asm__ __volatile__("pause");
    }
}