#include <kprint.h>
#include <task/execve.h>
#include <mm/frame.h>
#include <mm/page.h>
#include <task/task.h>
#include <irq/gate.h>

void task_to_user_mode(uint64_t addr, uint64_t load_start, uint64_t load_end, char **argv, char **envp)
{
    cli();

    uint64_t stack = (uint64_t)current_task->context;

    uint64_t backup_stack = stack;

    stack -= sizeof(struct pt_regs);

    struct pt_regs *iframe = (struct pt_regs *)stack;

    iframe->cs = SELECTOR_USER_CS;
    iframe->ss = SELECTOR_USER_DS;
    iframe->ds = SELECTOR_USER_DS;
    iframe->es = SELECTOR_USER_DS;
    iframe->rflags = (0 << 12) | (0b10) | (1 << 9);
    iframe->rip = addr;

    iframe->rdi = 0;
    if (argv != NULL)
    {
        int argc = 0;

        char **dst_argv = (char **)(iframe->rsp - sizeof(char **) * 8);
        uint64_t str_addr = (uint64_t)dst_argv;

        for (argc = 0; argc < 8 && argv[argc] != NULL; ++argc)
        {
            if (*argv[argc] = '\0')
                break;

            int argv_len = strlen(argv[argc]) + 1;
            strncpy((void *)str_addr, argv[argc], argv_len);
            str_addr -= argv_len;
            dst_argv[argc] = (char *)str_addr;
            ((char *)str_addr)[argv_len] = '\0';
        }

        iframe->rsp = str_addr - 8;
        iframe->rdi = argc;
        iframe->rsi = (uint64_t)dst_argv;
    }

    current_task->thread->gs = SELECTOR_USER_DS;
    current_task->thread->fs = SELECTOR_USER_DS;

    __asm__ __volatile__("movq %0, %%fs\n\t"
                         "movq %0, %%gs\n\t" ::"r"(current_task->thread->fs),
                         "r"(current_task->thread->gs));

    current_task->thread->elf_mapping_start = load_start;
    current_task->thread->elf_mapping_end = load_end;

    __asm__ __volatile__("movq %0, %%cr3\n\t" ::"r"(current_task->pgdir->table));

    page_map_range_to(get_current_page_dir(), USER_STACK_TOP - USER_STACK_SIZE, 0, USER_STACK_SIZE, USER_PTE_FLAGS);

    iframe->rbp = USER_STACK_TOP;
    iframe->rsp = USER_STACK_TOP;

    write_kgsbase((uint64_t)current_task);

    __asm__ __volatile__("movq %0, %%rsp\n\t"
                         "jmp ret_from_intr" ::"r"(iframe));
}

void load_module(struct limine_file *module, char **argv, char **envp)
{
    const Elf64_Ehdr *ehdr = (const Elf64_Ehdr *)module->address;

    uint64_t e_entry = ehdr->e_entry;

    if (e_entry == 0)
    {
        return;
    }

    // 验证ELF魔数
    if (memcmp((void *)ehdr->e_ident, "\x7F"
                                      "ELF",
               4) != 0)
    {
        kerror("Invalid ELF magic\n");
        return;
    }

    // 检查架构和类型
    if (ehdr->e_ident[4] != 2 ||   // 64-bit
        ehdr->e_machine != 0x3E || // x86_64
        (ehdr->e_type != 2))
    {
        kerror("Unsupported ELF format\n");
        return;
    }

    uint64_t load_start = UINT64_MAX;
    uint64_t load_end = 0;

    // 处理程序头
    const Elf64_Phdr *phdr = (const Elf64_Phdr *)((char *)ehdr + ehdr->e_phoff);
    for (int i = 0; i < ehdr->e_phnum; ++i)
    {
        if (phdr[i].p_type != PT_LOAD)
            continue;

        uint64_t seg_addr = phdr[i].p_vaddr;
        uint64_t seg_size = phdr[i].p_memsz;
        uint64_t file_size = phdr[i].p_filesz;
        uint64_t page_size = PAGE_SIZE;
        uint64_t page_mask = page_size - 1;

        // 计算对齐后的地址和大小
        uint64_t aligned_addr = seg_addr & ~page_mask;
        uint64_t size_diff = seg_addr - aligned_addr;
        uint64_t alloc_size = (seg_size + size_diff + page_mask) & ~page_mask;

        if (aligned_addr < load_start)
            load_start = aligned_addr;
        else if (aligned_addr + alloc_size > load_end)
            load_end = aligned_addr + alloc_size;

        page_directory_t *pgdir = get_current_page_dir();
        page_map_range_to(pgdir, aligned_addr, 0, alloc_size, USER_EXEC_PTE_FLAGS);

        // 复制数据
        memcpy((char *)aligned_addr + size_diff,
               (char *)ehdr + phdr[i].p_offset,
               file_size);

        // 清零剩余内存
        if (seg_size > file_size)
        {
            memset((char *)aligned_addr + size_diff + file_size,
                   0, seg_size - file_size);
        }
    }

    char buf[128];
    task_to_user_mode(e_entry, load_start, load_end, argv, envp);
}
