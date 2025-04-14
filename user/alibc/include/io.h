#pragma once

#include <stdint.h>
#include <stddef.h>

struct blkio_req
{
    uint64_t buf;
    uint64_t lba;
    uint64_t len;
    uint64_t flags;
};

// blkio_req->flags
#define BLKIO_WRITE (1UL << 0)

#define ICEIL(x, y) ((x) / (y) + ((x) % (y) != 0))

// 端口写（8位）
static inline void outb(uint16_t port, uint8_t value)
{
    __asm__ __volatile__("outb %1, %0" : : "dN"(port), "a"(value));
}

// 端口读（8位）
static inline uint8_t inb(uint16_t port)
{
    uint8_t ret;
    __asm__ __volatile__("inb %1, %0" : "=a"(ret) : "dN"(port));
    return ret;
}

// 端口写（16位）
static inline void outw(uint16_t port, uint16_t value)
{
    __asm__ __volatile__("outw %1, %0" : : "dN"(port), "a"(value));
}

// 端口读（16位）
static inline uint16_t inw(uint16_t port)
{
    uint16_t ret;
    __asm__ __volatile__("inw %1, %0" : "=a"(ret) : "dN"(port));
    return ret;
}

// 端口写（32位）
static inline void outl(uint16_t port, uint32_t value)
{
    __asm__ __volatile__("outl %1, %0" : : "dN"(port), "a"(value));
}

// 端口读（32位）
static inline uint32_t inl(uint16_t port)
{
    uint32_t ret;
    __asm__ __volatile__("inl %1, %0" : "=a"(ret) : "dN"(port));
    return ret;
}

static inline void port_delay(int counter)
{
    __asm__ __volatile__("   test %0, %0\n"
                         "   jz 1f\n"
                         "2: dec %0\n"
                         "   jnz 2b\n"
                         "1: dec %0" ::"a"(counter));
}
