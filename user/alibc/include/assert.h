#pragma once

#include <stdlib.h>

#define assert(cond) \
    do               \
    {                \
        if (!(cond)) \
        {            \
            abort(); \
        }            \
    } while (0)
