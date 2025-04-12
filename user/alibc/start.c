#include <stddef.h>

extern void exit(int code);
extern int main(int argc, char **argv);

void alibc_start()
{
    exit(main(0, NULL));

    while (1)
        __asm__ __volatile__("pause");
}
