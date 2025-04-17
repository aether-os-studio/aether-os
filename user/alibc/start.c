#include <stddef.h>

extern void exit(int code);
extern int main(int argc, char **argv);

extern void init_heap();

void alibc_start(int argc, char **argv)
{
    init_heap();

    exit(main(argc, argv));

    while (1)
        __asm__ __volatile__("pause");
}
