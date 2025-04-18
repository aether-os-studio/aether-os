#include <libsyscall.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    printf("init task is running\n");

    int child_pid = 0;
    int status;

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/acpid.exec", NULL, NULL);
    }
    else
    {
        waitpid(child_pid, &status);
    }

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/blkd.exec", NULL, NULL);
    }
    else
    {
        waitpid(child_pid, &status);
    }

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/pcid.exec", NULL, NULL);
    }
    else
    {
        waitpid(child_pid, &status);
    }

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/ps2d.exec", NULL, NULL);
    }
    else
    {
        waitpid(child_pid, &status);
    }

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/ahcid.exec", NULL, NULL);
    }
    else
    {
        waitpid(child_pid, &status);
    }

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/nvmed.exec", NULL, NULL);
    }
    else
    {
        waitpid(child_pid, &status);
    }

    child_pid = fork();
    if (child_pid == 0)
    {
        load_module("/usr/bin/fsd.exec", NULL, NULL);
    }
    else
    {
        waitpid(child_pid, &status);
    }

    int ret = execve("/usr/bin/shell.exec", NULL, NULL);
    if (ret < 0)
    {
        printf("execve shell failed\n");
    }

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}
