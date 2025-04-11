#pragma once

#include <klibc.h>
#include <mm/bitmap.h>

typedef struct
{
    Bitmap bitmap;
    size_t origin_frames;
    size_t usable_frames;
} FrameAllocator;

extern FrameAllocator frame_allocator;

void frame_init();

void free_frame(uint64_t addr);
uint64_t alloc_frames(size_t count);
