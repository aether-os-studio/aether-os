#pragma once

#include <klibc.h>

#define KERNEL_HEAP_START 0xFFFFFFFFC0000000
#define KERNEL_HEAP_SIZE (16 * 1024 * 1024)

void heap_init();

void memory_init(uint64_t start_addr, uint64_t size);
void *malloc(size_t size);
void free(void *ptr);
