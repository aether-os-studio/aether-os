#include <libsyscall.h>
#include <libdaemon.h>
#include <stdio.h>
#include "partition.h"
#include "fsd.h"
#include "vfs/dev.h"
#include "vfs/vfs.h"

#define MAX_TASK_NUM 2048

fs_t pid_to_fs[MAX_TASK_NUM];

uint64_t fsd_open(uint64_t pid, const char *name, uint64_t mode, uint64_t flags)
{
    fs_t *fs = &pid_to_fs[pid];
    vfs_node_t node = vfs_open(name);
    if (node == NULL)
    {
        return (uint64_t)-EBADF;
    }
    for (uint64_t i = 16; i < 16 + MAX_FD_NUM; i++)
    {
        if (fs->fds[i - 16].node == NULL)
        {
            fs->fds[i - 16].node = node;
            return i;
        }
    }

    return (uint64_t)-EFAULT;
}

uint64_t fsd_read(uint64_t pid, uint64_t fd, uint64_t buf, uint64_t len)
{
    fs_t *fs = &pid_to_fs[pid];
    if (fd >= MAX_FD_NUM)
    {
        return (uint64_t)-EFAULT;
    }
    vfs_node_t node = fs->fds[fd - 16].node;
    if (node == NULL)
    {
        return (uint64_t)-EBADF;
    }
    return vfs_read(node, (void *)buf, fs->fds[fd - 16].offset, len);
}

uint64_t fsd_write(uint64_t pid, uint64_t fd, uint64_t buf, uint64_t len)
{
    fs_t *fs = &pid_to_fs[pid];
    if (fd >= MAX_FD_NUM)
    {
        return (uint64_t)-EFAULT;
    }
    vfs_node_t node = fs->fds[fd - 16].node;
    if (node == NULL)
    {
        return (uint64_t)-EBADF;
    }
    return vfs_write(node, (void *)buf, fs->fds[fd - 16].offset, len);
}

uint64_t fsd_ioctl(uint64_t pid, uint64_t fd, uint64_t cmd, uint64_t arg)
{
    fs_t *fs = &pid_to_fs[pid];
    if (fd >= MAX_FD_NUM)
    {
        return (uint64_t)-EFAULT;
    }
    vfs_node_t node = fs->fds[fd - 16].node;
    if (node == NULL)
    {
        return (uint64_t)-EBADF;
    }
    if (node == NULL)
    {
        return (uint64_t)-EBADF;
    }
    switch (cmd)
    {
    case FSD_IOCTL_GETSIZE:
        return node->size;
        break;

    default:
        break;
    }
    return 0;
}

uint64_t fsd_lseek(uint64_t pid, uint64_t fd, uint64_t offset)
{
    fs_t *fs = &pid_to_fs[pid];
    if (fd >= MAX_FD_NUM)
    {
        return (uint64_t)-EFAULT;
    }
    fs->fds[fd - 16].offset = offset;
}

uint64_t fsd_readdir(uint64_t pid, uint64_t fd, uint64_t buf, uint64_t size)
{
    fs_t *fs = &pid_to_fs[pid];
    if (fd >= MAX_FD_NUM)
    {
        return (uint64_t)-EFAULT;
    }
    vfs_node_t node = fs->fds[fd - 16].node;
    if (node == NULL)
    {
        return (uint64_t)-EBADF;
    }

    uint64_t len = 0;
    dirent_t *dirents = (dirent_t *)buf;
    list_foreach(node->child, i)
    {
        if (len >= size)
            break;
        vfs_node_t child = (vfs_node_t)i->data;
        strncpy(dirents[len].name, child->name, 255);
        dirents[len].type = child->type;
        len++;
    }

    return len;
}

uint64_t fsd_chdir(uint64_t pid, const char *dirname)
{
    fs_t *fs = &pid_to_fs[pid];
    char buf[256];
    if (fs->cwd)
    {
        char *cwd = vfs_get_fullpath(fs->cwd);
        if (!strcmp(cwd, "/"))
        {
            sprintf(buf, "/%s", dirname);
        }
        else
        {
            sprintf(buf, "%s/%s", cwd, dirname);
        }
        free(cwd);
        vfs_close(fs->cwd);
    }
    else
    {
        sprintf(buf, "%s", dirname);
    }
    if (streq(dirname, "/"))
    {
        fs->cwd = rootdir;
        return 0;
    }
    vfs_node_t node = vfs_open(buf);
    if (!node)
    {
        return (uint64_t)-ENOENT;
    }
    if (node->type != file_dir)
    {
        vfs_close(node);
        return (uint64_t)-ENOTDIR;
    }
    fs->cwd = node;

    return 0;
}

uint64_t fsd_getcwd(uint64_t pid, char *cwd)
{
    fs_t *fs = &pid_to_fs[pid];
    if (fs->cwd == NULL)
    {
        return (uint64_t)-ENOENT;
    }
    char *now_cwd = vfs_get_fullpath(fs->cwd);
    strncpy(cwd, now_cwd, 255);
    free(now_cwd);
    return 0;
}

uint64_t fsd_close(uint64_t pid, uint64_t fd)
{
    fs_t *fs = &pid_to_fs[pid];
    if (fd >= MAX_FD_NUM)
    {
        return (uint64_t)-EFAULT;
    }
    vfs_node_t node = fs->fds[fd - 16].node;
    vfs_close(node);
    fs->fds[fd - 16].node = NULL;
    fs->fds[fd - 16].offset = 0;

    return 0;
}

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

user_scheme_command_t fsd_command;

uint64_t fsd_daemon(daemon_t *daemon)
{
    printf("fs daemon is running\n");

    partition_init();

    vfs_init();

    dev_init();

    iso9660_init();
    fatfs_init();

    mount_root();

    memset(&fsd_command, 0, sizeof(user_scheme_command_t));

    regist_fsd((uint64_t)&fsd_command);

    memset(pid_to_fs, 0, sizeof(pid_to_fs));

    finish_daemon(daemon);

    while (1)
    {
        if (fsd_command.cmd != 0)
        {
            uint64_t result = 0;

            switch (fsd_command.cmd)
            {
            case FSD_OPEN:
                result = fsd_open(fsd_command.d, (const char *)fsd_command.a, fsd_command.b, fsd_command.c);
                break;
            case FSD_READ:
                result = fsd_read(fsd_command.d, fsd_command.a, fsd_command.b, fsd_command.c);
                break;
            case FSD_WRITE:
                result = fsd_write(fsd_command.d, fsd_command.a, fsd_command.b, fsd_command.c);
                break;
            case FSD_IOCTL:
                result = fsd_ioctl(fsd_command.d, fsd_command.a, fsd_command.b, fsd_command.c);
                break;
            case FSD_LSEEK:
                result = fsd_lseek(fsd_command.d, fsd_command.a, fsd_command.b);
                break;
            case FSD_READDIR:
                result = fsd_readdir(fsd_command.d, fsd_command.a, fsd_command.b, fsd_command.c);
                break;
            case FSD_CHDIR:
                result = fsd_chdir(fsd_command.d, (const char *)fsd_command.a);
                break;
            case FSD_GETCWD:
                result = fsd_getcwd(fsd_command.d, (char *)fsd_command.a);
                break;
            case FSD_CLOSE:
                result = fsd_close(fsd_command.d, fsd_command.a);
                break;
            default:
                printf("unknown command\n");
                break;
            }

            fsd_command.a = result;

            fsd_command.cmd = 0;
        }

        __asm__ __volatile__("pause");
    }
}

int main()
{
    return start_daemon(fsd_daemon);
}
