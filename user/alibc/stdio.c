#include <stdio.h>
#include <stdarg.h>
#include <libsyscall.h>

char getchar()
{
    char buf[1];
    read(0, buf, 1);
    return buf[0];
}

char buf[4096];

extern int vsprintf(char *buf, const char *fmt, va_list args);

int printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    int len = vsprintf(buf, fmt, args);

    va_end(args);

    write(1, buf, len);

    return len;
}

int sprintf(char *buf, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    int len = vsprintf(buf, fmt, args);

    va_end(args);

    return len;
}
