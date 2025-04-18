#include <libsyscall.h>
#include <stdio.h>

int main()
{
    printf("shell is running\n");

    int fd = open("/usr/bin", 0, 0);
    dirent_t buf[128];
    int num = getdents(fd, buf, 128);

    for (int i = 0; i < num; i++)
    {
        printf("file %s\ttype %s\n", buf[i].name, buf[i].type == file_dir ? "dir" : "file");
    }

    close(fd);

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}
