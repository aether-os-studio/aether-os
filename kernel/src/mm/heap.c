#include <mm/page.h>
#include <mm/heap.h>

void heap_init()
{
    page_map_range_to(get_kernel_page_dir(), KERNEL_HEAP_START, 0, KERNEL_HEAP_SIZE, KERNEL_PTE_FLAGS);

    memory_init(KERNEL_HEAP_START, KERNEL_HEAP_SIZE);
}

typedef struct heap_block
{
    size_t size;
    bool free;
    struct heap_block *next;
    struct heap_block *prev;
} heap_block_t;

#define HEAP_BLOCK_HEADER_SIZE sizeof(heap_block_t)
#define HEAP_MIN_SIZE (4 * 1024)
#define HEAP_ALIGNMENT 8

static heap_block_t *heap_start = NULL;
static heap_block_t *heap_end = NULL;
static uint64_t heap_base_addr = 0;
static uint64_t heap_total_size = 0;

void memory_init(uint64_t start_addr, uint64_t size)
{
    if (size < HEAP_MIN_SIZE)
        return;

    // Ensure the start address is properly aligned
    start_addr = (start_addr + HEAP_ALIGNMENT - 1) & ~(HEAP_ALIGNMENT - 1);

    heap_base_addr = start_addr;
    heap_total_size = size;

    heap_start = (heap_block_t *)start_addr;
    heap_start->size = size - HEAP_BLOCK_HEADER_SIZE;
    heap_start->free = true;
    heap_start->next = NULL;
    heap_start->prev = NULL;
    heap_end = heap_start;
}

void *malloc(size_t size)
{
    if (size == 0 || heap_start == NULL)
        return NULL;

    // Align the requested size
    size = (size + HEAP_ALIGNMENT - 1) & ~(HEAP_ALIGNMENT - 1);

    heap_block_t *current = heap_start;
    while (current)
    {
        // Verify block is within heap bounds
        if ((uint64_t)current < heap_base_addr ||
            (uint64_t)current >= heap_base_addr + heap_total_size)
        {
            return NULL; // Corrupted heap
        }

        if (current->free)
        {
            // Check if current block can fit the requested size + header
            if (current->size >= size)
            {
                // Check if we can split the block
                if (current->size > size + HEAP_BLOCK_HEADER_SIZE + HEAP_ALIGNMENT)
                {
                    heap_block_t *new_block = (heap_block_t *)((char *)current + HEAP_BLOCK_HEADER_SIZE + size);

                    // Verify new block is within bounds
                    if ((uint64_t)new_block >= heap_base_addr + heap_total_size)
                    {
                        // Can't split - just use the whole block
                        current->free = false;
                        return (void *)((char *)current + HEAP_BLOCK_HEADER_SIZE);
                    }

                    // Initialize new block
                    new_block->size = current->size - size - HEAP_BLOCK_HEADER_SIZE;
                    new_block->free = true;
                    new_block->next = current->next;
                    new_block->prev = current;

                    if (current->next)
                        current->next->prev = new_block;
                    current->next = new_block;

                    // Update current block size
                    current->size = size;

                    // Update heap_end if we're at the end
                    if (current == heap_end)
                        heap_end = new_block;
                }

                current->free = false;
                return (void *)((char *)current + HEAP_BLOCK_HEADER_SIZE);
            }
        }
        current = current->next;
    }
    return NULL; // No suitable block found
}

void free(void *ptr)
{
    if (ptr == NULL || heap_start == NULL)
        return;

    // Get the block header and verify it's within heap bounds
    heap_block_t *block = (heap_block_t *)((char *)ptr - HEAP_BLOCK_HEADER_SIZE);
    if ((uint64_t)block < heap_base_addr ||
        (uint64_t)block >= heap_base_addr + heap_total_size)
    {
        return; // Invalid pointer
    }

    // Mark as free
    block->free = true;

    // Merge with next block if free
    if (block->next && block->next->free)
    {
        // Verify next block is within bounds
        if ((uint64_t)block->next >= heap_base_addr &&
            (uint64_t)block->next < heap_base_addr + heap_total_size)
        {
            block->size += block->next->size + HEAP_BLOCK_HEADER_SIZE;
            block->next = block->next->next;
            if (block->next)
                block->next->prev = block;
            else
                heap_end = block;
        }
    }

    // Merge with previous block if free
    if (block->prev && block->prev->free)
    {
        // Verify previous block is within bounds
        if ((uint64_t)block->prev >= heap_base_addr &&
            (uint64_t)block->prev < heap_base_addr + heap_total_size)
        {
            block->prev->size += block->size + HEAP_BLOCK_HEADER_SIZE;
            block->prev->next = block->next;
            if (block->next)
                block->next->prev = block->prev;
            else
                heap_end = block->prev;
        }
    }
}