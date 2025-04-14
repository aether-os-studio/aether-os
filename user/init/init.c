#include <libsyscall.h>
#include <stdio.h>

int main()
{
    printf("init task is running\n");

    int child_pid = 0;
    int status;

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/acpid.exec");
        exit(-1);
    }
    else
    {
        waitpid(child_pid, &status);
    }

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/pcid.exec");
        exit(-1);
    }
    else
    {
        waitpid(child_pid, &status);
    }

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/ahcid.exec");
        exit(-1);
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
