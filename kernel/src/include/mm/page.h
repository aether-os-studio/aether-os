#pragma once

#define PTE_PRESENT (0x1UL << 0)
#define PTE_WRITEABLE (0x1UL << 1)
#define PTE_USER (0x1UL << 2)
#define PTE_HUGE (0x1UL << 7)
#define PTE_NO_EXECUTE (0x1UL << 63)

#define KERNEL_PTE_FLAGS (PTE_PRESENT | PTE_WRITEABLE | PTE_NO_EXECUTE)
#define USER_EXEC_PTE_FLAGS (PTE_PRESENT | PTE_WRITEABLE | PTE_USER)
#define USER_PTE_FLAGS (USER_EXEC_PTE_FLAGS | PTE_NO_EXECUTE)

#define USER_BRK_START 0x700000000000
#define USER_BRK_END 0x7FFFFFFFFFFF

#define PAGE_SIZE 0x1000

#include <klibc.h>

typedef struct page_table_entry
{
    uint64_t value;
} __attribute__((packed)) page_table_entry_t;

typedef struct
{
    page_table_entry_t entries[512];
} __attribute__((packed)) page_table_t;

typedef struct page_directory
{
    page_table_t *table;
} page_directory_t;

static inline void flush_tlb(uint64_t addr)
{
    __asm__ volatile("invlpg (%0)" ::"r"(addr) : "memory");
};

uint64_t get_cr3();
page_directory_t *get_kernel_page_dir();
page_directory_t *get_current_page_dir();
void page_map_to(page_directory_t *directory, uint64_t addr, uint64_t frame, uint64_t flags);
void page_map_range_to(page_directory_t *directory, uint64_t addr, uint64_t frame, uint64_t size, uint64_t flags);
void copy_page_table_recursive(page_table_t *source_table, page_table_t *new_table, int level);
void free_page_table_recursive(page_directory_t *directory, int level);
uint64_t translate_addr(page_directory_t *directory, uint64_t vaddr);
page_directory_t *clone_directory(page_directory_t *src);
void page_init();
