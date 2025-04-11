#pragma once

#include <klibc.h>

#define IA32_FS_BASE 0xc0000100

#define IA32_GS_BASE 0xc0000101

extern uint64_t (*read_fsbase)();
extern void (*write_fsbase)(uint64_t value);
extern uint64_t (*read_gsbase)();
extern void (*write_gsbase)(uint64_t value);

static inline bool has_fsgsbase()
{
    uint32_t eax, ebx, ecx, edx;
    __asm__ __volatile__("cpuid" : "=a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx) : "a"(0x07), "c"(0x00));
    return (ebx & (1 << 0));
}

uint64_t fsgsbase_init();
