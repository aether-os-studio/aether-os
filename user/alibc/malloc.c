#include <libsyscall.h>
#include <stdlib.h>

#define align_to(size, align) (((size) + (align) - 1) & ~((align) - 1))
#define BLOCK_HEADER_SIZE sizeof(struct block_header)
#define ALIGNMENT 8

// 内存块头部结构
struct block_header
{
    size_t size;
    int free;
    struct block_header *next;
};

static struct block_header *head = NULL; // 空闲链表头
static void *current_break = NULL;       // 当前堆顶地址

// 自定义brk系统调用封装
static void *sys_brk(void *addr)
{
    return (void *)enter_syscall(SYS_BRK, (uint64_t)addr, 0, 0, 0, 0, 0);
}

// 请求堆扩展
static struct block_header *request_memory(size_t size)
{
    void *old_break = current_break;
    void *new_break = old_break + size;
    void *actual_break = sys_brk(new_break);

    if (actual_break >= new_break)
    {
        current_break = new_break;
        struct block_header *block = (struct block_header *)old_break;
        block->size = size;
        block->free = 0;
        block->next = NULL;
        return block;
    }
    return NULL; // 扩展失败
}

// 分割块
static void split_block(struct block_header *block, size_t size)
{
    size_t remaining_size = block->size - size;
    if (remaining_size > BLOCK_HEADER_SIZE)
    {
        struct block_header *new_block = (struct block_header *)((char *)block + size);
        new_block->size = remaining_size - BLOCK_HEADER_SIZE;
        new_block->free = 1;
        new_block->next = block->next;
        block->next = new_block;
        block->size = size;
    }
}

// 合并相邻空闲块
static void coalesce(struct block_header *block)
{
    // 合并后面的块
    struct block_header *next_block = block->next;
    if (next_block && (char *)block + block->size == (char *)next_block)
    {
        if (block->free && next_block->free)
        {
            block->size += BLOCK_HEADER_SIZE + next_block->size;
            block->next = next_block->next;
        }
    }

    // 合并前面的块
    struct block_header *prev = NULL;
    struct block_header *current = head;
    while (current && current != block)
    {
        prev = current;
        current = current->next;
    }
    if (prev && (char *)prev + prev->size + BLOCK_HEADER_SIZE == (char *)block)
    {
        if (prev->free && block->free)
        {
            prev->size += BLOCK_HEADER_SIZE + block->size;
            prev->next = block->next;
        }
    }
}

// 分配内存
void *malloc(size_t size)
{
    if (size == 0)
        return NULL;

    if (current_break == NULL)
    {
        current_break = (void *)sys_brk(0);
    }

    size = align_to(size, ALIGNMENT);
    size_t total_size = BLOCK_HEADER_SIZE + size;

    struct block_header *block = NULL;

    // 查找空闲块
    struct block_header **prev_ptr = &head;
    for (struct block_header *curr = head; curr; prev_ptr = &curr->next, curr = curr->next)
    {
        if (curr->free && curr->size >= total_size)
        {
            block = curr;
            *prev_ptr = curr->next; // 从空闲链表中移除
            break;
        }
    }

    if (block)
    {
        split_block(block, total_size);
        block->free = 0;
    }
    else
    {
        block = request_memory(total_size);
        if (!block)
            return NULL;
        block->next = head;
        head = block;
    }

    return (void *)(block + 1);
}

// 释放内存
void free(void *ptr)
{
    if (!ptr)
        return;

    struct block_header *block = (struct block_header *)ptr - 1;
    block->free = 1;

    // 插入到空闲链表头部以便快速合并
    block->next = head;
    head = block;

    coalesce(block);
}
