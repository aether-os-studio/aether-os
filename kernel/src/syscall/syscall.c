#include <syscall/syscall.h>
#include <kprint.h>
#include <task/task.h>
#include <irq/irq.h>
#include <irq/gate.h>
#include <task/signal.h>
#include <acpi/acpi.h>
#include <mm/hhdm.h>
#include <scheme/scheme.h>
#include <mm/frame.h>
#include <task/kbd.h>
#include <fs/fs.h>

uint64_t sys_exit(uint64_t code)
{
    task_exit((int)code);

    return 0xFFFFFFFFFFFFFFFF;
}

uint64_t sys_physmap(uint64_t addr, uint64_t size, uint64_t flags)
{
    if (addr >= get_physical_memory_offset() || size == 0)
    {
        return (uint64_t)-EINVAL;
    }

    page_directory_t *current_page_dir = get_current_page_dir();

    uint64_t vaddr = USER_MAPPING_SPACE + addr;

    uint64_t pte_flags = 0;

    pte_flags |= (PTE_PRESENT | PTE_USER);

    if (flags & PROT_READ)
    {
    }

    if (flags & PROT_WRITE)
    {
        pte_flags |= PTE_WRITEABLE;
    }

    if (!(flags & PROT_EXEC))
    {
        pte_flags |= PTE_NO_EXECUTE;
    }

    page_map_range_to(current_page_dir, vaddr, addr, size, pte_flags);

    return vaddr;
}

uint64_t sys_brk(uint64_t addr)
{
    page_directory_t *current_page_dir = get_current_page_dir();

    uint64_t new_brk = (addr + PAGE_SIZE - 1) & (~(PAGE_SIZE - 1));

    if (new_brk == 0)
        return current_task->brk_start;
    if (new_brk < current_task->brk_end)
        return 0;

    uint64_t start = current_task->brk_end;
    uint64_t size = new_brk - current_task->brk_end;

    page_map_range_to(current_page_dir, start, 0, PAGE_SIZE * 8, USER_PTE_FLAGS);

    new_brk = start + size;

    current_task->brk_end = new_brk;

    return new_brk;
}

uint64_t sys_sbrk(uint64_t size)
{
    uint64_t retval = current_task->brk_end;

    if (size == 0)
        return retval;

    page_map_range_to(get_current_page_dir(), retval, 0, size, USER_PTE_FLAGS);

    current_task->brk_end = retval + size;

    return retval;
}

extern volatile struct limine_framebuffer_request framebuffer_request;

void sys_get_info(bootstrap_info_t *info)
{
    info->rsdp_phys_address = rsdp_paddr;

    struct limine_framebuffer *framebuffer = framebuffer_request.response->framebuffers[0];

    info->framebuffer_address = (uint64_t)framebuffer->address;
    info->framebuffer_height = framebuffer->width;
    info->framebuffer_height = framebuffer->height;

    info->red_mask_shift = framebuffer->red_mask_shift;
    info->red_mask_size = framebuffer->red_mask_size;
    info->blue_mask_shift = framebuffer->blue_mask_shift;
    info->blue_mask_size = framebuffer->blue_mask_size;
    info->green_mask_shift = framebuffer->green_mask_shift;
    info->green_mask_size = framebuffer->green_mask_size;
}

uint64_t sys_open(const char *name, uint64_t mode, uint64_t flags)
{
    uint64_t i;

    if (!memcmp(name, "/scheme/", 8))
    {
        for (i = 3; i < MAX_FD_NUM; i++)
        {
            if (current_task->schemes[i] == NULL)
                break;
        }

        if (i >= MAX_FD_NUM)
        {
            return (uint64_t)-EFAULT;
        }

        scheme_t *scheme = scheme_open(name);
        if (!scheme)
            return (uint64_t)-ENOENT;

        current_task->schemes[i] = scheme;

        return i;
    }
    else
    {
        return fsd_open(name, mode, flags);
    }
}

uint64_t sys_read(uint64_t fd, uint64_t buf, uint64_t len)
{
    if (fd < MAX_FD_NUM)
    {
        if (fd == 0 && len <= 128)
        {
            uint64_t write_len = 0;
            while (write_len < len)
            {
                uint8_t scancode = get_keyboard_input();
                if (scancode != 0)
                {
                    ((uint8_t *)buf)[write_len] = scancode;
                    write_len++;
                }
                else
                {
                    __asm__ __volatile__("int %0" ::"i"(APIC_TIMER_INTERRUPT_VECTOR));
                }
            }

            return len;
        }

        scheme_t *scheme = current_task->schemes[fd];
        if (scheme == NULL)
        {
            return (uint64_t)-EBADF;
        }

        return scheme_read(scheme, buf, len);
    }
    else
    {
        return fsd_read(fd, buf, len);
    }
}

extern int sys_printk(const char *str, uint64_t len);

uint64_t sys_write(uint64_t fd, uint64_t buf, uint64_t len)
{
    if (fd < MAX_FD_NUM)
    {
        if (fd == 1 || fd == 2)
        {
            return printk((const char *)buf);
        }

        scheme_t *scheme = current_task->schemes[fd];
        if (scheme == NULL)
        {
            return (uint64_t)-EBADF;
        }

        return scheme_write(scheme, buf, len);
    }
    else
    {
        return fsd_write(fd, buf, len);
    }
}

uint64_t sys_lseek(uint64_t fd, uint64_t offset)
{
    if (fd < MAX_FD_NUM)
    {
        scheme_t *scheme = current_task->schemes[fd];
        if (scheme == NULL)
        {
            return (uint64_t)-EBADF;
        }

        scheme->offset = offset;

        return offset;
    }
    else
    {
        return fsd_lseek(fd, offset);
    }
}

uint64_t sys_ioctl(uint64_t fd, uint64_t cmd, uint64_t arg)
{
    if (fd < MAX_FD_NUM)
    {
        scheme_t *scheme = current_task->schemes[fd];
        if (scheme == NULL)
        {
            return (uint64_t)-EBADF;
        }

        return scheme_ioctl(scheme, cmd, arg);
    }
    else
    {
        return fsd_ioctl(fd, cmd, arg);
    }
}

uint64_t sys_close(uint64_t fd)
{
    if (fd < MAX_FD_NUM)
    {
        if (!current_task->schemes[fd])
            return (uint64_t)-EBADF;

        scheme_close(current_task->schemes[fd]);
        current_task->schemes[fd] = (scheme_t *)NULL;
    }
    else
    {
        return fsd_close(fd);
    }
}

uint64_t sys_getdents(uint64_t fd, uint64_t buf, uint64_t size)
{
    if (fd < MAX_FD_NUM)
    {
        if (!current_task->schemes[fd])
            return (uint64_t)-EBADF;

        return scheme_readdir(current_task->schemes[fd], buf, size);
    }
    else
    {
        return fsd_readdir(fd, buf, size);
    }
}

uint64_t sys_chdir(const char *dirname)
{
    return fsd_chdir(dirname);
}

uint64_t sys_getcwd(char *cwd)
{
    return fsd_getcwd(cwd);
}

extern void sys_load_module(const char *, char **, char **);
extern uint64_t sys_execve(const char *, char **, char **);

void *real_memcpy(void *dst, void *src, long len)
{
    return memcpy(dst, src, len);
}

uint64_t switch_to_kernel_stack()
{
    return current_task->syscall_stack;
}

void syscall_handler(struct pt_regs *regs, struct pt_regs *user_regs)
{
    regs->rip = regs->rcx;
    regs->rflags = regs->r11;
    regs->cs = SELECTOR_USER_CS;
    regs->ss = SELECTOR_USER_DS;
    regs->rsp = (uint64_t)(user_regs + 1);

    uint64_t idx = regs->rax;

    uint64_t arg1 = regs->rdi;
    uint64_t arg2 = regs->rsi;
    uint64_t arg3 = regs->rdx;
    uint64_t arg4 = regs->r10;
    uint64_t arg5 = regs->r8;
    uint64_t arg6 = regs->r9;

    switch (idx)
    {
    case SYS_EXIT:
        regs->rax = sys_exit(arg1);
        break;
    case SYS_GETPID:
        regs->rax = current_task->task_id;
        break;
    case SYS_GETPPID:
        regs->rax = current_task->parent_task_id;
        break;
    case SYS_FORK:
        regs->rax = sys_fork(regs);
        break;
    case SYS_WAITPID:
        regs->rax = sys_waitpid(arg1, (int *)arg2);
        break;

    case SYS_SIGNAL:
        regs->rax = sys_signal(arg1, arg2, arg3);
        break;
    case SYS_SIGACTION:
        regs->rax = sys_sigaction(arg1, (sigaction_t *)arg2, (sigaction_t *)arg3);
        break;
    case SYS_SETMASK:
        regs->rax = sys_ssetmask(arg1);
        break;
    case SYS_SENDSIGNAL:
        sys_sendsignal(arg1, arg2);
        regs->rax = 0;
        break;

    case SYS_LOAD_MODULE:
        sys_load_module((const char *)arg1, (char **)arg2, (char **)arg3);
        regs->rax = 0;
        break;
    case SYS_EXECVE:
        regs->rax = sys_execve((const char *)arg1, (char **)arg2, (char **)arg3);
        break;

    case SYS_IOPL:
        sys_iopl(arg1);
        regs->rax = 0;
        break;

    case SYS_GET_INFO:
        sys_get_info((bootstrap_info_t *)arg1);
        regs->rax = 0;
        break;

    case SYS_BRK:
        regs->rax = sys_brk(arg1);
        break;
    case SYS_SBRK:
        regs->rax = sys_sbrk(arg1);
        break;
    case SYS_PHYSMAP:
        regs->rax = sys_physmap(arg1, arg2, arg3);
        break;
    case SYS_VIRTTOPHYS:
        regs->rax = translate_addr(get_current_page_dir(), arg1);
        break;

    case SYS_SCHEME_CREATE:
        scheme_create((const char *)arg1, arg2);
        regs->rax = 0;
        break;

    case SYS_OPEN:
        regs->rax = sys_open((const char *)arg1, arg2, arg3);
        break;
    case SYS_READ:
        regs->rax = sys_read(arg1, arg2, arg3);
        break;
    case SYS_WRITE:
        regs->rax = sys_write(arg1, arg2, arg3);
        break;
    case SYS_CLOSE:
        regs->rax = sys_close(arg1);
        break;
    case SYS_LSEEK:
        regs->rax = sys_lseek(arg1, arg2);
        break;
    case SYS_IOCTL:
        regs->rax = sys_ioctl(arg1, arg2, arg3);
        break;
    case SYS_CHDIR:
        regs->rax = sys_chdir((const char *)arg1);
        break;
    case SYS_GETCWD:
        regs->rax = sys_getcwd((char *)arg1);
        break;
    case SYS_GETDENTS:
        regs->rax = sys_getdents(arg1, arg2, arg3);
        break;

    // TODO: 权限检测
    case SYS_DMA_ALLOC:
        regs->rax = alloc_frames(arg1);
        break;

    case SYS_DMA_FREE:
        free_frames(arg1, arg2);
        regs->rax = 0;
        break;

    case SYS_REGIST_FSD:
        sys_regist_fs(arg1);
        regs->rax = 0;
        break;

    default:
        regs->rax = ((uint64_t)-ENOSYS);
        break;
    }
}

// MSR寄存器地址定义
#define MSR_EFER 0xC0000080         // EFER MSR寄存器
#define MSR_STAR 0xC0000081         // STAR MSR寄存器
#define MSR_LSTAR 0xC0000082        // LSTAR MSR寄存器
#define MSR_SYSCALL_MASK 0xC0000084 // SYSCALL_MASK MSR寄存器

void syscall_init()
{
    uint64_t efer;

    // 1. 启用 EFER.SCE (System Call Extensions)
    efer = rdmsr(MSR_EFER);
    efer |= 1; // 设置 SCE 位
    wrmsr(MSR_EFER, efer);

    uint16_t cs_sysret_cmp = SELECTOR_USER_CS - 16;
    uint16_t ss_sysret_cmp = SELECTOR_USER_DS - 8;
    uint16_t cs_syscall_cmp = SELECTOR_KERNEL_CS;
    uint16_t ss_syscall_cmp = SELECTOR_KERNEL_DS - 8;

    if (cs_sysret_cmp != ss_sysret_cmp)
    {
        kerror("Sysret offset is not valid (1)\n");
        return;
    }

    if (cs_syscall_cmp != ss_syscall_cmp)
    {
        kerror("Syscall offset is not valid (2)\n");
        return;
    }

    // 2. 设置 STAR MSR
    uint64_t star = 0;
    star = ((uint64_t)(SELECTOR_USER_DS - 8) << 48) | // SYSRET 的基础 CS
           ((uint64_t)SELECTOR_KERNEL_CS << 32);      // SYSCALL 的 CS
    wrmsr(MSR_STAR, star);

    // 3. 设置 LSTAR MSR (系统调用入口点)
    wrmsr(MSR_LSTAR, (uint64_t)syscall_exception);

    // 4. 设置 SYSCALL_MASK MSR (RFLAGS 掩码)
    wrmsr(MSR_SYSCALL_MASK, (1 << 9));
}
