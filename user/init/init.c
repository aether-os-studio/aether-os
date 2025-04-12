#include "libsyscall.h"

void signal_handler()
{
    write(1, "SIGNAL_HANDLER\n", 16);

    // signal(SIGINT, signal_handler);
}

int main()
{
    write(1, "Hello World\n", 12);

    signal(SIGINT, signal_handler);

    write(1, "Sending SIGNAL\n", 16);

    // 测试，发送信号给自己
    uint64_t pid = getpid();
    send_signal(pid, SIGINT);

    write(1, "Sending done.\n", 16);

    while (1)
    {
        __asm__ __volatile__("pause");
    }

    return 0;
}
