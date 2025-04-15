#include <libsyscall.h>
#include <stdio.h>
#include <stdlib.h>

int main()
{
    printf("init task is running\n");

    int child_pid = 0;
    int status;

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/acpid.exec");
    }
    else
    {
        waitpid(child_pid, &status);
    }

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/blkd.exec");
    }
    else
    {
        waitpid(child_pid, &status);
    }

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/pcid.exec");
    }
    else
    {
        waitpid(child_pid, &status);
    }

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/ahcid.exec");
    }
    else
    {
        waitpid(child_pid, &status);
    }

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/fsd.exec");
    }
    else
    {
        waitpid(child_pid, &status);
    }

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}
