#include <libsyscall.h>
#include <libdaemon.h>
#include <stdio.h>

uint64_t fsd_daemon(daemon_t *daemon)
{
    int fd = open("/scheme/block/0", 0, 0);
    if (fd < 0)
    {
        printf("Cannot open /scheme/block/0");
        return fd;
    }

    int blksize = ioctl(fd, SCHEME_IOCTL_GETBLKSIZE, 0);

    printf("blkdev 0 blksize = %d\n", blksize);

    uint8_t *buf = (uint8_t *)malloc(blksize);
    memset(buf, 0, blksize);
    lseek(fd, blksize * 1);
    int ret = read(fd, buf, blksize);
    if (ret != blksize)
    {
        printf("Cannot read blkdev");
        close(fd);
        return ret;
    }

    finish_daemon(daemon);

    return 0;
}

int main()
{
    start_daemon(fsd_daemon);
}
