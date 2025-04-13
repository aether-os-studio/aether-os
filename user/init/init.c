#include "libsyscall.h"

int sig_tested;

void signal_handler()
{
    write(1, "SIGNAL_HANDLER\n", 16);

    sig_tested = 1;

    // signal(SIGINT, signal_handler);
}

int main()
{
    write(1, "init task is running\n", 22);

    sig_tested = 0;

    signal(SIGALRM, (uint64_t)signal_handler);

    write(1, "Sending SIGNAL\n", 16);

    // 测试，发送信号给自己
    uint64_t pid = getpid();
    send_signal(pid, SIGALRM);

    write(1, "Sending done.\n", 16);

    while (!sig_tested)
    {
        __asm__ __volatile__("pause");
    }

    int child_pid = fork();
    if (child_pid == 0)
    {
        write(1, "Child process\n", 14);
    }
    else
    {
        write(1, "Parent process\n", 15);
    }

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}
