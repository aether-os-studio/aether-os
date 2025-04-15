#include <libsyscall.h>
#include <libdaemon.h>
#include <stdio.h>

#define is_digit(c) ((c) >= '0' && (c) <= '9') // 用来判断是否是数字的宏

#define MAX_BLKDEV_NUM 8
#define DEV_SCHEME_NAME_MAX 128

typedef struct blkdev
{
    char dev_scheme_name[DEV_SCHEME_NAME_MAX];
    uint64_t block_size;
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

        uint64_t start = offset;
        uint64_t end = offset + len;

        uint64_t start_sector_read_start = offset % dev->block_size;

        uint64_t start_sector_id = start / dev->block_size;
        uint64_t end_sector_id = (end - 1) / dev->block_size;

        uint64_t buffer_size = (end_sector_id - start_sector_id + 1) * dev->block_size;

        uint8_t *tmp = malloc(buffer_size);

        lseek(fd, start_sector_id * dev->block_size);

        int ret = read(fd, (void *)tmp, buffer_size);

        memcpy(buf, tmp + start_sector_read_start, len);

        free(tmp);

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

        uint64_t start = offset;
        uint64_t end = offset + len;

        uint64_t start_sector_read_start = offset % dev->block_size;

        uint64_t start_sector_id = start / dev->block_size;
        uint64_t end_sector_id = (end - 1) / dev->block_size;

        uint64_t buffer_size = (end_sector_id - start_sector_id + 1) * dev->block_size;

        uint8_t *tmp = malloc(buffer_size);

        lseek(fd, start_sector_id * dev->block_size);

        int ret = read(fd, (void *)tmp, buffer_size);

        memcpy(tmp + start_sector_read_start, (void *)buf, len);

        ret = write(fd, tmp, buffer_size);

        free(tmp);

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

        strcpy(newdev->dev_scheme_name, target_name);
        newdev->block_size = arg;

        blk_devnum++;
    }
    break;

    default:
        break;
    }
}

uint64_t blkd_daemon(daemon_t *daemon)
{
    blk_devnum = 0;

    printf("blk daemon is running\n");

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

        __asm__ __volatile__("pause");
    }
}

int main()
{
    return start_daemon(blkd_daemon);
}
