#pragma once

#include <klibc.h>

enum
{
    SYS_READ = 1,
    SYS_WRITE,
    SYS_OPEN,
    SYS_CLOSE,
    SYS_LSEEK,
    SYS_IOCTL,
    SYS_CHDIR,
    SYS_GETCWD,
    SYS_GETDENTS,
    SYS_SIGACTION,
    SYS_SIGNAL,
    SYS_SETMASK,
    SYS_SENDSIGNAL,
    SYS_EXIT,
    SYS_GETPID,
    SYS_GETPPID,
    SYS_FORK,
    SYS_WAITPID,
    SYS_LOAD_MODULE,
    SYS_EXECVE,
    SYS_IOPL,

    SYS_GET_INFO,

    SYS_BRK,
    SYS_SBRK,
    SYS_PHYSMAP,
    SYS_VIRTTOPHYS,

    SYS_SCHEME_CREATE,

    SYS_DMA_ALLOC,
    SYS_DMA_FREE,

    SYS_REGIST_FSD,

    SYS_NUM,
};

typedef struct bootstrap_info
{
    // About ACPI
    uint64_t rsdp_phys_address;
    // About framebuffer
    uint64_t framebuffer_address;
    uint64_t framebuffer_width;
    uint64_t framebuffer_height;
    uint8_t red_mask_size;
    uint8_t red_mask_shift;
    uint8_t green_mask_size;
    uint8_t green_mask_shift;
    uint8_t blue_mask_size;
    uint8_t blue_mask_shift;
} bootstrap_info_t;

#define PROT_READ (1UL << 0)
#define PROT_WRITE (1UL << 1)
#define PROT_EXEC (1UL << 2)

#define USER_MAPPING_SPACE 0x0

#define USER_SPACE_BUFFER_MAPPING_OFFSET 0x600000000000

extern void syscall_exception();

void syscall_init();
