#pragma once

#include <klibc.h>
#include <mm/bitmap.h>

#define MAX_USABLE_REGIONS_COUNT 64

typedef struct
{
    Bitmap bitmap;
    size_t origin_frames;
    size_t usable_frames;
} FrameAllocator;

extern FrameAllocator frame_allocator;

void frame_init();

void free_frames(uint64_t addr, uint64_t size);
uint64_t alloc_frames(size_t count);
