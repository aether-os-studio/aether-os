#include <libsyscall.h>
#include <libdaemon.h>
#include <stdio.h>
#include "partition.h"
#include "vfs/dev.h"
#include "vfs/vfs.h"

extern void iso9660_init();
extern void fatfs_init();

void mount_root()
{
    printf("Mounting root\n");
    bool err = true;
    for (uint64_t i = 0; i < partition_num; i++)
    {
        char buf[11];
        sprintf(buf, "/dev/part%d", i);
        if (vfs_mount(buf, rootdir))
            err = true;
        else
        {
            err = false;
            break;
        }
    }
    if (err)
        printf("Mount root failed\n");
    else
        printf("Mount root OK\n");
}

uint64_t fsd_daemon(daemon_t *daemon)
{
    printf("fs daemon is running\n");

    partition_init();

    vfs_init();

    dev_init();

    iso9660_init();
    fatfs_init();

    mount_root();

    finish_daemon(daemon);

    // test time!
    list_foreach(rootdir->child, node)
    {
        vfs_node_t file = (vfs_node_t)node->data;
        printf("%s\n", file->name);
    };

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}

int main()
{
    start_daemon(fsd_daemon);
}
