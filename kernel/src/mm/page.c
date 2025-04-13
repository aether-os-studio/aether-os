#include <mm/hhdm.h>
#include <mm/frame.h>
#include <mm/page.h>
#include <mm/heap.h>
#include <task/task.h>

page_directory_t kernel_page_dir;

uint64_t get_cr3()
{
    uint64_t cr3;
    __asm__ __volatile__("movq %%cr3, %0" : "=r"(cr3));
    return cr3;
}

page_directory_t *get_kernel_page_dir()
{
    return &kernel_page_dir;
}

void page_init()
{
    kernel_page_dir.table = (page_table_t *)get_cr3();
}

void page_table_clear(page_table_t *table)
{
    for (int i = 0; i < 512; i++)
    {
        table->entries[i].value = 0;
    }
}

page_table_t *page_table_create(page_table_entry_t *entry, bool user)
{
    if (entry->value == (uint64_t)NULL)
    {
        uint64_t frame = alloc_frames(1);
        entry->value = frame | PTE_PRESENT | PTE_WRITEABLE | (user ? PTE_USER : 0);
        page_table_t *table = (page_table_t *)phys_to_virt(entry->value & 0x00007fffffff000UL);
        page_table_clear(table);
        return table;
    }
    page_table_t *table = (page_table_t *)phys_to_virt(entry->value & 0x00007fffffff000UL);
    return table;
}

page_directory_t *get_current_page_dir()
{
    return current_task->pgdir;
}

void page_map_to(page_directory_t *directory, uint64_t addr, uint64_t frame, uint64_t flags)
{
    uint64_t l4_index = (((addr >> 39)) & 0x1FF);
    uint64_t l3_index = (((addr >> 30)) & 0x1FF);
    uint64_t l2_index = (((addr >> 21)) & 0x1FF);
    uint64_t l1_index = (((addr >> 12)) & 0x1FF);

    bool user = (flags & PTE_USER) != 0;

    page_table_t *l4_table = phys_to_virt(directory->table);
    page_table_t *l3_table = page_table_create(&(l4_table->entries[l4_index]), user);
    page_table_t *l2_table = page_table_create(&(l3_table->entries[l3_index]), user);
    page_table_t *l1_table = page_table_create(&(l2_table->entries[l2_index]), user);

    l1_table->entries[l1_index].value = (frame & 0x00007fffffff000) | flags;

    flush_tlb(addr);
}

void page_map_range_to(page_directory_t *directory, uint64_t addr, uint64_t frame, uint64_t size, uint64_t flags)
{
    addr = addr & (~0xFFFUL);
    frame = frame & (~0xFFFUL);

    uint64_t paddr = frame;
    for (uint64_t vaddr = addr; vaddr < addr + size; vaddr += PAGE_SIZE)
    {
        if (frame == 0)
        {
            paddr = alloc_frames(1);
        }
        page_map_to(directory, vaddr, paddr, flags);

        paddr += PAGE_SIZE;
    }
}

static bool is_huge_page(page_table_entry_t *entry)
{
    return (((uint64_t)entry->value) & PTE_HUGE) != 0;
}

void copy_page_table_recursive(page_table_t *source_table, page_table_t *new_table, int level)
{
    int max = 512;
    if (level == 4)
    {
        memcpy(phys_to_virt((uint64_t *)new_table) + 256, phys_to_virt((uint64_t *)source_table + 256), PAGE_SIZE / 2);
        max = 256;
    }
    for (int i = 0; i < max; i++)
    {
        page_table_entry_t *entry = phys_to_virt(&source_table->entries[i]);

        if (entry->value == 0 || is_huge_page(entry))
        {
            phys_to_virt(new_table)->entries[i].value = entry->value;
            continue;
        }

        if (level == 1 && entry->value & PTE_PRESENT)
        {
            uint64_t phys = alloc_frames(1);
            phys_to_virt(new_table)->entries[i].value = phys | (entry->value & (~0x00007fffffff000UL));
            uint64_t dst = phys_to_virt(phys);
            uint64_t src = phys_to_virt(entry->value & 0x00007fffffff000UL);
            memcpy((void *)dst, (void *)src, PAGE_SIZE);
            continue;
        }

        uint64_t frame = alloc_frames(1);
        page_table_t *new_page_table = (page_table_t *)frame;
        phys_to_virt(new_table)->entries[i].value = frame | (entry->value & (~0x00007fffffff000UL));

        page_table_t *source_page_table_next = (page_table_t *)(entry->value & 0x00007fffffff000UL);
        page_table_t *target_page_table_next = new_page_table;

        copy_page_table_recursive(source_page_table_next, target_page_table_next, level - 1);
    }
}

page_directory_t *clone_directory(page_directory_t *src)
{
    page_directory_t *new_directory = (page_directory_t *)malloc(sizeof(page_directory_t));
    if (new_directory == NULL)
        return NULL;
    new_directory->table = (page_table_t *)alloc_frames(1);
    copy_page_table_recursive(src->table, new_directory->table, 4);
    return new_directory;
}
