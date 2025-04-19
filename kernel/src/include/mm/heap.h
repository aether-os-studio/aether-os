#pragma once

#include <klibc.h>

#define MIN_ALLOC_SZ 4

#define MIN_WILDERNESS 0x2000
#define MAX_WILDERNESS 0x1000000

#define BIN_COUNT 16
#define BIN_MAX_IDX (BIN_COUNT - 1)

typedef unsigned int uint;

typedef struct node_t
{
    uint hole;
    uint size;
    struct node_t *next;
    struct node_t *prev;
} node_t;

typedef struct
{
    node_t *header;
} footer_t;

typedef struct
{
    node_t *head;
} bin_t;

typedef struct
{
    uint64_t start;
    uint64_t end;
    bin_t *bins[BIN_COUNT];
} heap_t;

static uint overhead = sizeof(footer_t) + sizeof(node_t);

#define KERNEL_HEAP_START 0xFFFFFFFFC0000000
#define KERNEL_HEAP_SIZE (16 * 1024 * 1024)

void heap_init();

void *heap_alloc(heap_t *heap, size_t size);
void heap_free(heap_t *heap, void *p);
uint expand(heap_t *heap, size_t sz);
void contract(heap_t *heap, size_t sz);

uint get_bin_index(size_t sz);
void create_foot(node_t *head);
footer_t *get_foot(node_t *head);

node_t *get_wilderness(heap_t *heap);

void *malloc(size_t size);
void free(void *ptr);
