#pragma once

#include "printk.h"

#define ksuccess(...)        \
    do                       \
    {                        \
        printk("[ ");        \
        printk("SUCCESS");   \
        printk(" ] ");       \
        printk(__VA_ARGS__); \
        printk("\n");        \
    } while (0);

#define kinfo(...)           \
    do                       \
    {                        \
        printk("[ INFO ] "); \
        printk(__VA_ARGS__); \
        printk("\n");        \
    } while (0);

#define kdebug(...)           \
    do                        \
    {                         \
        printk("[ DEBUG ] "); \
        printk(__VA_ARGS__);  \
        printk("\n");         \
    } while (0);

#define kwarn(...)           \
    do                       \
    {                        \
        printk("[ ");        \
        printk("WARN");      \
        printk(" ] ");       \
        printk(__VA_ARGS__); \
        printk("\n");        \
    } while (0);

#define kerror(...)          \
    do                       \
    {                        \
        printk("[ ");        \
        printk("ERROR");     \
        printk(" ] ");       \
        printk(__VA_ARGS__); \
        printk("\n");        \
    } while (0);

#define kterminated(...)      \
    do                        \
    {                         \
        printk("[ ");         \
        printk("TERMINATED"); \
        printk(" ] ");        \
        printk(__VA_ARGS__);  \
        printk("\n");         \
    } while (0);

#define kBUG(...)            \
    do                       \
    {                        \
        printk("[ ");        \
        printk("BUG");       \
        printk(" ] ");       \
        printk(__VA_ARGS__); \
        printk("\n");        \
    } while (0);
