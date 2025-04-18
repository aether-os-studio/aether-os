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
        page_table_t *table = (page_table_t *)phys_to_virt(entry->value & 0x000ffffffffff000UL);
        page_table_clear(table);
        return table;
    }
    page_table_t *table = (page_table_t *)phys_to_virt(entry->value & 0x000ffffffffff000UL);
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

    if (l1_table->entries[l1_index].value != 0)
        return;

    l1_table->entries[l1_index].value = (frame & 0x000ffffffffff000UL) | flags;

    flush_tlb(addr);
}

void page_map_range_to(page_directory_t *directory, uint64_t addr, uint64_t frame, uint64_t size, uint64_t flags)
{
    addr = addr & (~(PAGE_SIZE - 1));
    frame = frame & (~(PAGE_SIZE - 1));
    size = (size + PAGE_SIZE - 1) & (~(PAGE_SIZE - 1));

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

// 递归释放页表及其子页表
static void free_page_table(uint64_t phys_addr, int level)
{
    uint64_t *table = phys_to_virt((uint64_t *)phys_addr);

    for (int i = 0; i < 512; ++i)
    {
        uint64_t pte = table[i];
        if (!(pte & PTE_PRESENT))
            continue; // 跳过无效项

        if (level == 1)
        { // PT层，释放4KB页
            free_frames(pte & 0x00007FFFFFFFF000, 1);
        }
        else
        {
            free_page_table(pte & 0x00007FFFFFFFF000, level - 1); // 递归子页表
        }
    }

    free_frames(phys_addr, 1); // 释放当前页表页
}

// 主函数：释放用户空间所有页表及物理页
void free_page_table_recursive(page_directory_t *directory, int level)
{
    uint64_t *pml4 = phys_to_virt((uint64_t *)directory->table);

    for (int i = 0; i < 256; ++i)
    {
        if (pml4[i] & PTE_PRESENT)
        {
            free_page_table(pml4[i] & 0x00007FFFFFFFF000, level - 1);
            pml4[i] = 0;
        }
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

uint64_t translate_addr(page_directory_t *directory, uint64_t vaddr)
{
    uint64_t offset = vaddr & 0xFFFUL;

    uint64_t *page_table_addr = phys_to_virt((uint64_t *)directory->table);

    uint64_t *pml4 = (uint64_t *)page_table_addr;
    uint64_t pml4_id = (vaddr >> 39) & 0x1FFUL;
    if ((pml4[pml4_id] & (1 << 0)) == 0)
    {
        return 0;
    }
    uint64_t *pdpt = (uint64_t *)phys_to_virt(pml4[pml4_id] & ~(0xFFFUL));
    uint64_t pdpt_id = (vaddr >> 30) & 0x1FFUL;
    if ((pdpt[pdpt_id] & (1 << 0)) == 0)
    {
        return 0;
    }
    uint64_t *pd = (uint64_t *)phys_to_virt(pdpt[pdpt_id] & ~(0xFFFUL));
    uint64_t pd_id = (vaddr >> 21) & 0x1FFUL;
    if (pd[pd_id] & PTE_HUGE)
    {
        return (pd[pd_id] & ~((1UL << 21) - 1)) + (vaddr & ((1UL << 21) - 1));
    }
    if ((pd[pd_id] & (1 << 0)) == 0)
    {
        return 0;
    }
    uint64_t *pt = (uint64_t *)phys_to_virt(pd[pd_id] & ~(0xFFFUL));
    uint64_t pt_id = (vaddr >> 12) & 0x1FFUL;
    if ((pt[pt_id] & (1 << 0)) == 0)
    {
        return 0;
    }

    return (pt[pt_id] & 0x00007FFFFFFFF000) + offset;
}
