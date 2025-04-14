#pragma once

#include <klibc.h>
#include <task/task.h>

#define SCHEME_NAME_MAX 128

typedef struct scheme
{
    char name[SCHEME_NAME_MAX];
    char target_name[SCHEME_NAME_MAX];
    uint64_t offset;
    uint64_t command_d;
    task_t *task;
    void *user_scheme;
} scheme_t;

enum
{
    SCHEME_COMMAND_READ = 1,
    SCHEME_COMMAND_WRITE,
    SCHEME_COMMAND_IOCTL,
};

typedef struct user_scheme_command
{
    uint64_t cmd;
    uint64_t a;
    uint64_t b;
    uint64_t c;
    uint64_t d;
    uint64_t e;
    uint64_t f;
} user_scheme_command_t;

typedef struct user_scheme
{
    user_scheme_command_t command;
} user_scheme_t;

void scheme_init();

void scheme_create(const char *name, uint64_t ptr);

scheme_t *scheme_open(const char *name);
void scheme_close(scheme_t *scheme);
uint64_t scheme_read(scheme_t *scheme, uint64_t buffer, uint64_t len);
uint64_t scheme_write(scheme_t *scheme, uint64_t buffer, uint64_t len);
uint64_t scheme_ioctl(scheme_t *scheme, uint64_t buffer, uint64_t len);
