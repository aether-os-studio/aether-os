#pragma once

#include <stdint.h>
#include <stddef.h>

// 端口写（8位）
void outb(uint16_t port, uint8_t value)
{
    __asm__ __volatile__("outb %1, %0" : : "dN"(port), "a"(value));
}

// 端口读（8位）
uint8_t inb(uint16_t port)
{
    uint8_t ret;
    __asm__ __volatile__("inb %1, %0" : "=a"(ret) : "dN"(port));
    return ret;
}

// 端口写（16位）
void outw(uint16_t port, uint16_t value)
{
    __asm__ __volatile__("outw %1, %0" : : "dN"(port), "a"(value));
}

// 端口读（16位）
uint16_t inw(uint16_t port)
{
    uint16_t ret;
    __asm__ __volatile__("inw %1, %0" : "=a"(ret) : "dN"(port));
    return ret;
}

// 端口写（32位）
void outl(uint16_t port, uint32_t value)
{
    __asm__ __volatile__("outl %1, %0" : : "dN"(port), "a"(value));
}

// 端口读（32位）
uint32_t inl(uint16_t port)
{
    uint32_t ret;
    __asm__ __volatile__("inl %1, %0" : "=a"(ret) : "dN"(port));
    return ret;
}
