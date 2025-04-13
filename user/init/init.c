#include "libsyscall.h"

int main()
{
    write(1, "init task is running\n", 22);

    int child_pid = fork();
    if (child_pid == 0)
    {
        // write(1, "Child process\n", 14);
        load_module("/usr/bin/acpid.exec");
    }
    else
    {
        int status;
        waitpid(child_pid, &status);
        write(1, "Child process exited\n", 24);
    }

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}
