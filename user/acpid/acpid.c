#include <libdaemon.h>
#include <stdio.h>

uint64_t acpid_daemon(daemon_t *daemon)
{
    printf("acpid daemon is running\n");

    finish_daemon(daemon);

    return 0;
}

int main()
{
    return start_daemon(acpid_daemon);
}
