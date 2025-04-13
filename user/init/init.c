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
        // write(1, "Child process\n", 14);
        load_module("/usr/bin/acpid.exec");
    }
    else
    {
        waitpid(child_pid, &status);
        printf("acpid process exited, status = %d\n", status);
    }

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}
