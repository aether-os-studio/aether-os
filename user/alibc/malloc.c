#include <libsyscall.h>
#include <stdlib.h>

#define align_to(size, align) (((size) + (align) - 1) & ~((align) - 1))
#define BLOCK_HEADER_SIZE sizeof(struct block_header)
#define ALIGNMENT 8

struct block_header
{
    size_t size;
    int free;
    struct block_header *next;
};

static struct block_header *head = NULL;
static void *current_break = NULL;

static void *sys_brk(void *addr)
{
    return (void *)enter_syscall(SYS_BRK, (uint64_t)addr, 0, 0, 0, 0, 0);
}

static struct block_header *request_memory(size_t size)
{
    void *old_break = current_break;
    if (!old_break)
    {
        old_break = sys_brk(0);
        current_break = old_break;
    }

    void *new_break = (char *)old_break + size;
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
    return NULL;
}

static void split_block(struct block_header *block, size_t size)
{
    size_t remaining_size = block->size - size;
    if (remaining_size > BLOCK_HEADER_SIZE + ALIGNMENT)
    {
        struct block_header *new_block = (struct block_header *)((char *)block + size);
        new_block->size = remaining_size - BLOCK_HEADER_SIZE;
        new_block->free = 1;
        new_block->next = block->next;
        block->next = new_block;
        block->size = size;

        // 将新分割出的空闲块插入空闲链表
        if (new_block->free)
        {
            new_block->next = head;
            head = new_block;
        }
    }
}

static void coalesce()
{
    struct block_header *curr = head;
    struct block_header *prev = NULL;

    while (curr)
    {
        // 合并后面的块
        if (curr->next && (char *)curr + curr->size == (char *)curr->next)
        {
            if (curr->free && curr->next->free)
            {
                curr->size += BLOCK_HEADER_SIZE + curr->next->size;
                curr->next = curr->next->next;
                continue; // 继续检查是否有更多可合并的块
            }
        }

        // 合并前面的块
        if (prev && (char *)prev + prev->size == (char *)curr - BLOCK_HEADER_SIZE)
        {
            if (prev->free && curr->free)
            {
                prev->size += BLOCK_HEADER_SIZE + curr->size;
                prev->next = curr->next;
                curr = prev; // 重置当前指针继续检查
                continue;
            }
        }

        prev = curr;
        curr = curr->next;
    }
}

void *malloc(size_t size)
{
    if (size == 0)
        return NULL;

    size = align_to(size, ALIGNMENT);
    size_t total_size = BLOCK_HEADER_SIZE + size;

    // 第一次调用时初始化堆
    if (!current_break)
    {
        current_break = sys_brk(0);
    }

    struct block_header *best_fit = NULL;
    struct block_header **best_fit_prev = &head;

    // 使用最佳适配算法
    for (struct block_header **curr = &head; *curr; curr = &(*curr)->next)
    {
        if ((*curr)->free && (*curr)->size >= total_size)
        {
            if (!best_fit || (*curr)->size < best_fit->size)
            {
                best_fit = *curr;
                best_fit_prev = curr;
            }
        }
    }

    if (best_fit)
    {
        *best_fit_prev = best_fit->next; // 从空闲链表中移除
        split_block(best_fit, total_size);
        best_fit->free = 0;
        return (void *)(best_fit + 1);
    }

    // 没有找到合适块，请求新内存
    struct block_header *block = request_memory(total_size);
    if (!block)
        return NULL;

    return (void *)(block + 1);
}

void free(void *ptr)
{
    if (!ptr)
        return;

    struct block_header *block = (struct block_header *)ptr - 1;
    block->free = 1;

    // 插入到空闲链表头部
    block->next = head;
    head = block;

    // 合并相邻空闲块
    coalesce();
}

void *realloc(void *ptr, size_t size)
{
    if (!ptr)
        return malloc(size);
    if (size == 0)
    {
        free(ptr);
        return NULL;
    }

    struct block_header *block = (struct block_header *)ptr - 1;
    size_t aligned_size = align_to(size, ALIGNMENT);

    // 尝试就地扩展
    if (block->size >= aligned_size + BLOCK_HEADER_SIZE)
    {
        split_block(block, aligned_size + BLOCK_HEADER_SIZE);
        return ptr;
    }

    // 尝试合并后面的块
    if (block->next && block->next->free &&
        (block->size + BLOCK_HEADER_SIZE + block->next->size) >= aligned_size)
    {
        block->size += BLOCK_HEADER_SIZE + block->next->size;
        block->next = block->next->next;
        split_block(block, aligned_size + BLOCK_HEADER_SIZE);
        return ptr;
    }

    // 无法就地扩展，分配新内存并复制数据
    void *new_ptr = malloc(size);
    if (!new_ptr)
        return NULL;

    size_t copy_size = block->size < size ? block->size : size;
    memcpy(new_ptr, ptr, copy_size);
    free(ptr);
    return new_ptr;
}
