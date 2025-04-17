#include <libsyscall.h>
#include <libdaemon.h>
#include <io.h>
#include <stdio.h>

#define PORT_KB_DATA 0x60
#define PORT_KB_STATUS 0x64
#define PORT_KB_CMD 0x64

#define KBCMD_WRITE_CMD 0x60
#define KBCMD_READ_CMD 0x20

#define KB_INIT_MODE 0x47

#define KBSTATUS_IBF 0x02
#define KBSTATUS_OBF 0x01

#define wait_KB_write() \
    while (inb(PORT_KB_STATUS) & KBSTATUS_IBF)

#define wait_KB_read() \
    while (inb(PORT_KB_STATUS) & KBSTATUS_OBF)

uint64_t ps2d_daemon(daemon_t *daemon)
{
    printf("ps2 daemon is running\n");

    iopl(3);

    wait_KB_write();
    outb(PORT_KB_CMD, KBCMD_WRITE_CMD);
    wait_KB_read();
    outb(PORT_KB_DATA, KB_INIT_MODE);

    finish_daemon(daemon);

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}

int main()
{
    return start_daemon(ps2d_daemon);
}
