#include <stddef.h>

extern void exit(int code);
extern int main(int argc, char **argv);

extern void init_heap();

void alibc_start()
{
    init_heap();

    exit(main(0, NULL));

    while (1)
        __asm__ __volatile__("pause");
}
