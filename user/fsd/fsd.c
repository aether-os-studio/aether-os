#include <libsyscall.h>
#include <libdaemon.h>
#include <stdio.h>

uint64_t fsd_daemon(daemon_t *daemon)
{
    printf("fs daemon is running\n");

    int fd = open("/scheme/block/0", 0, 0);
    uint8_t *buf = malloc(1);
    memset(buf, 0, 1);

    lseek(fd, 512);

    read(fd, buf, 1);
    buf[0] = 0xFF;
    write(fd, buf, 1);

    close(fd);

    finish_daemon(daemon);

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}

int main()
{
    start_daemon(fsd_daemon);
}
