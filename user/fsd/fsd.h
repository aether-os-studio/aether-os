#pragma once

#include <libsyscall.h>
#include "vfs/vfs.h"

#define MAX_FD_NUM 32

typedef struct inner_fd
{
    vfs_node_t node;
    uint64_t offset;
} fd_t;

typedef struct fs
{
    char name[SCHEME_NAME_MAX];
    fd_t fds[MAX_FD_NUM];
    vfs_node_t cwd;
} fs_t;
