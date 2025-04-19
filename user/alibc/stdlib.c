#include <stdlib.h>

char *strdup(const char *s)
{
    size_t len = strlen((char *)s);
    char *ptr = (char *)malloc(len + 1);
    if (ptr == NULL)
        return NULL;
    memcpy(ptr, (void *)s, len + 1);
    return ptr;
}

void *memmove(void *dst, void *src, int count)
{
    void *ret = dst;
    if (dst <= src || (char *)dst >= ((char *)src + count))
    {
        while (count--)
        {
            *(char *)dst = *(char *)src;
            dst = (char *)dst + 1;
            src = (char *)src + 1;
        }
    }
    else
    {

        dst = (char *)dst + count - 1;
        src = (char *)src + count - 1;
        while (count--)
        {
            *(char *)dst = *(char *)src;
            dst = (char *)dst - 1;
            src = (char *)src - 1;
        }
    }
    return (ret);
}
