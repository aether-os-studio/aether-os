#include <libdaemon.h>

uint64_t daemon_ready;

void daemon_signal_handler()
{
    daemon_ready = 1;
}

uint64_t start_daemon(uint64_t (*func)(daemon_t *))
{
    int pid = fork();
    daemon_t daemon;
    if (pid == 0)
    {
        uint64_t ret = func(&daemon);

        return 0;
    }
    else
    {
        signal(SIGTERM, (uint64_t)daemon_signal_handler);
        while (!daemon_ready)
        {
            __asm__ __volatile__("pause");
        }

        return 0;
    }
}

void finish_daemon(daemon_t *daemon)
{
    send_signal(getppid(), SIGTERM);
}
