#include <libsyscall.h>
#include <stdio.h>

int main()
{
    printf("shell is running\n");

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}
