#include <mm/hhdm.h>
#include <mm/frame.h>
#include <mm/page.h>

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
    kernel_page_dir.table = get_cr3();
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
        entry->value = frame | (user ? (PTE_PRESENT | PTE_WRITEABLE | PTE_USER) : (PTE_PRESENT | PTE_WRITEABLE));
        page_table_t *table = (page_table_t *)phys_to_virt(entry->value & 0x000fffffffff000);
        page_table_clear(table);
        return table;
    }
    page_table_t *table = (page_table_t *)phys_to_virt(entry->value & 0x000fffffffff000);
    return table;
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
    uint64_t paddr = frame;
    for (uint64_t vaddr = addr; vaddr < addr + size; vaddr += PAGE_SIZE)
    {
        if (frame == 0)
        {
            paddr = alloc_frames(1);
        }
        else
        {
            paddr += PAGE_SIZE;
        }
        page_map_to(directory, vaddr, paddr, flags);
    }
}
