#include <libsyscall.h>
#include <libdaemon.h>
#include <stdio.h>

#define is_digit(c) ((c) >= '0' && (c) <= '9') // 用来判断是否是数字的宏

#define MAX_BLKDEV_NUM 8
#define DEV_SCHEME_NAME_MAX 128

typedef struct blkdev
{
    char dev_scheme_name[DEV_SCHEME_NAME_MAX];
} blkdev_t;

blkdev_t blk_devs[MAX_BLKDEV_NUM];
uint64_t blk_devnum;

user_scheme_t blkd_scheme;

uint64_t blkd_read(uint64_t buf, uint64_t len, uint64_t offset, char *target_name)
{
    // 只支持MAX_BLKDEV_NUM个块设备
    if (is_digit(target_name[0]))
    {
        uint8_t idx = target_name[0] - '0';
        if (idx >= MAX_BLKDEV_NUM)
        {
            return (uint64_t)-EINVAL;
        }

        blkdev_t *dev = &blk_devs[idx];

        int fd = open(dev->dev_scheme_name, 0, 0);
        if (fd < 0)
        {
            printf("Cannot get blkdev scheme");
            close(fd);
            return (uint64_t)-EBADF;
        }

        int ret = read(fd, (void *)buf, len);

        close(fd);

        return ret;
    }

    return 0;
}

uint64_t blkd_write(uint64_t buf, uint64_t len, uint64_t offset, char *target_name)
{
    // 只支持MAX_BLKDEV_NUM个块设备
    if (is_digit(target_name[0]))
    {
        uint8_t idx = target_name[0] - '0';
        if (idx >= MAX_BLKDEV_NUM)
        {
            return (uint64_t)-EINVAL;
        }

        blkdev_t *dev = &blk_devs[idx];

        int fd = open(dev->dev_scheme_name, 0, 0);
        if (fd < 0)
        {
            printf("Cannot get blkdev scheme");
            close(fd);
            return (uint64_t)-EBADF;
        }

        int ret = write(fd, (void *)buf, len);

        close(fd);

        return ret;
    }

    return 0;
}

uint64_t blkd_ioctl(uint64_t cmd, uint64_t arg, uint64_t offset, char *target_name)
{
    switch (cmd)
    {
    case SCHEME_IOCTL_GETBLKSIZE:
    {
        // 只支持MAX_BLKDEV_NUM个块设备
        if (is_digit(target_name[0]))
        {
            uint8_t idx = target_name[0] - '0';
            if (idx >= MAX_BLKDEV_NUM)
            {
                return (uint64_t)-EINVAL;
            }

            blkdev_t *dev = &blk_devs[idx];

            int fd = open(dev->dev_scheme_name, 0, 0);
            if (fd < 0)
            {
                printf("Cannot get blkdev scheme");
                close(fd);
                return (uint64_t)-EBADF;
            }

            int ret = ioctl(fd, cmd, arg);

            close(fd);

            return ret;
        }
    }
    break;
    case SCHEME_IOCTL_GETSIZE:
        return blk_devnum;
    case SCHEME_IOCTL_REGIST_BLKDEV:
    {
        char *blkdev_name = target_name;

        printf("regist blkdev %s\n", blkdev_name);

        blkdev_t *newdev = &blk_devs[blk_devnum];
        blk_devnum++;
        strcpy(newdev->dev_scheme_name, target_name);
    }
    break;

    default:
        break;
    }
}

uint64_t blkd_daemon(daemon_t *daemon)
{
    blk_devnum = 0;

    printf("blkd daemon is running\n");

    init_scheme(&blkd_scheme);

    scheme_create("/scheme/block", &blkd_scheme);

    finish_daemon(daemon);

    while (1)
    {
        if (blkd_scheme.command.cmd != 0)
        {
            uint64_t result = 0;
            switch (blkd_scheme.command.cmd)
            {
            case SCHEME_COMMAND_READ:
                result = blkd_read(blkd_scheme.command.a, blkd_scheme.command.b, blkd_scheme.command.c, (char *)blkd_scheme.command.d);
                break;
            case SCHEME_COMMAND_WRITE:
                result = blkd_write(blkd_scheme.command.a, blkd_scheme.command.b, blkd_scheme.command.c, (char *)blkd_scheme.command.d);
                break;
            case SCHEME_COMMAND_IOCTL:
                result = blkd_ioctl(blkd_scheme.command.a, blkd_scheme.command.b, blkd_scheme.command.c, (char *)blkd_scheme.command.d);
                break;
            default:
                printf("unknown command\n");
                break;
            }

            blkd_scheme.command.a = result;
            blkd_scheme.command.cmd = 0;
        }
    }
}

int main()
{
    return start_daemon(blkd_daemon);
}
