#pragma once

#include <klibc.h>
#include <scheme/scheme.h>

enum
{
    FSD_IOCTL_GETSIZE,
};

enum
{
    FSD_OPEN = 1,
    FSD_READ,
    FSD_WRITE,
    FSD_IOCTL,
    FSD_LSEEK,
    FSD_CLOSE,
};

void sys_regist_fs(uint64_t arg);

uint64_t fsd_open(const char *name, uint64_t mode, uint64_t flags);
uint64_t fsd_read(uint64_t fd, uint64_t buf, uint64_t len);
uint64_t fsd_write(uint64_t fd, uint64_t buf, uint64_t len);
uint64_t fsd_ioctl(uint64_t fd, uint64_t cmd, uint64_t arg);
uint64_t fsd_lseek(uint64_t fd, uint64_t offset);
uint64_t fsd_close(uint64_t fd);
