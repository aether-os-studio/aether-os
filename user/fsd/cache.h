#pragma once

#include <libsyscall.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_BLOCK_DEVICES_NUM 8

typedef struct block_device
{
    int fd;
    uint8_t *buf;
    uint64_t block_size;
    uint64_t (*read_from)(struct block_device *self, uint64_t lba, void *buffer, uint64_t size);
    uint64_t (*write_to)(struct block_device *self, uint64_t lba, void *buffer, uint64_t size);
} block_device_t;

void cache_init();

uint64_t block_read(uint64_t dev, uint64_t offset, void *buffer, uint64_t size);
uint64_t block_write(uint64_t dev, uint64_t offset, void *buffer, uint64_t size);