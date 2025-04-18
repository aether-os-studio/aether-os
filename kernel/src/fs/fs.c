#include <fs/fs.h>
#include <task/task.h>
#include <mm/hhdm.h>
#include <mm/heap.h>
#include <irq/irq.h>
#include <syscall/syscall.h>

spinlock_t fs_op_lock;
page_directory_t *fsd_pgdir = NULL;
user_scheme_command_t *fsd_user_command_addr = NULL;
uint64_t fsd_pid = 0;

void sys_regist_fs(uint64_t arg)
{
    spin_init(&fs_op_lock);

    fsd_pid = current_task->task_id;
    fsd_pgdir = current_task->pgdir;
    fsd_user_command_addr = phys_to_virt((user_scheme_command_t *)translate_addr(current_task->pgdir, arg));
}

uint64_t wait_command()
{
    while (fsd_user_command_addr->cmd != 0)
    {
        sti();

        __asm__ __volatile__("int %0" ::"i"(APIC_TIMER_INTERRUPT_VECTOR));
    }

    cli();

    spin_unlock(&fs_op_lock);

    return fsd_user_command_addr->a;
}

uint64_t fsd_open(const char *name, uint64_t mode, uint64_t flags)
{
    spin_lock(&fs_op_lock);

    uint64_t name_phys = translate_addr(get_current_page_dir(), (uint64_t)name);
    uint64_t name_virt = USER_SPACE_BUFFER_MAPPING_OFFSET + name_phys;
    page_map_range_to(fsd_pgdir, name_virt, name_phys, strlen(name), USER_PTE_FLAGS);
    if (name_phys == 0)
    {
        return (uint64_t)-EINVAL;
    }
    fsd_user_command_addr->a = name_virt;
    fsd_user_command_addr->b = mode;
    fsd_user_command_addr->c = flags;
    fsd_user_command_addr->d = current_task->task_id;
    fsd_user_command_addr->cmd = FSD_OPEN;

    return wait_command();
}

uint64_t fsd_read(uint64_t fd, uint64_t buf, uint64_t len)
{
    spin_lock(&fs_op_lock);

    fsd_user_command_addr->a = fd;
    uint64_t buf_phys = translate_addr(get_current_page_dir(), (uint64_t)buf);
    uint64_t buf_virt = USER_SPACE_BUFFER_MAPPING_OFFSET + buf_phys;
    page_map_range_to(fsd_pgdir, buf_virt, buf_phys, len, USER_PTE_FLAGS);
    if (buf_phys == 0)
    {
        return (uint64_t)-EINVAL;
    }
    fsd_user_command_addr->b = buf_virt;
    fsd_user_command_addr->c = len;
    fsd_user_command_addr->d = current_task->task_id;
    fsd_user_command_addr->cmd = FSD_READ;

    return wait_command();
}

uint64_t fsd_write(uint64_t fd, uint64_t buf, uint64_t len)
{
    spin_lock(&fs_op_lock);

    fsd_user_command_addr->a = fd;
    uint64_t buf_phys = translate_addr(get_current_page_dir(), (uint64_t)buf);
    uint64_t buf_virt = USER_SPACE_BUFFER_MAPPING_OFFSET + buf_phys;
    if (buf_phys == 0)
    {
        return (uint64_t)-EINVAL;
    }
    page_map_range_to(fsd_pgdir, buf_virt, buf_phys, len, USER_PTE_FLAGS);
    fsd_user_command_addr->b = buf_virt;
    fsd_user_command_addr->c = len;
    fsd_user_command_addr->d = current_task->task_id;
    fsd_user_command_addr->cmd = FSD_WRITE;

    return wait_command();
}

uint64_t fsd_ioctl(uint64_t fd, uint64_t cmd, uint64_t arg)
{
    spin_lock(&fs_op_lock);

    fsd_user_command_addr->a = fd;
    fsd_user_command_addr->b = cmd;
    fsd_user_command_addr->c = arg;
    fsd_user_command_addr->d = current_task->task_id;
    fsd_user_command_addr->cmd = FSD_IOCTL;

    return wait_command();
}

uint64_t fsd_lseek(uint64_t fd, uint64_t offset)
{
    spin_lock(&fs_op_lock);

    fsd_user_command_addr->a = fd;
    fsd_user_command_addr->b = offset;
    fsd_user_command_addr->d = current_task->task_id;
    fsd_user_command_addr->cmd = FSD_LSEEK;

    return wait_command();
}

uint64_t fsd_readdir(uint64_t fd, uint64_t buf, uint64_t size)
{
    spin_lock(&fs_op_lock);

    fsd_user_command_addr->a = fd;
    uint64_t buf_phys = translate_addr(get_current_page_dir(), (uint64_t)buf);
    uint64_t buf_virt = USER_SPACE_BUFFER_MAPPING_OFFSET + buf_phys;
    if (buf_phys == 0)
    {
        return (uint64_t)-EINVAL;
    }
    page_map_range_to(fsd_pgdir, buf_virt, buf_phys, size * sizeof(dirent_t), USER_PTE_FLAGS);
    fsd_user_command_addr->b = buf_virt;
    fsd_user_command_addr->c = size;
    fsd_user_command_addr->d = current_task->task_id;
    fsd_user_command_addr->cmd = FSD_READDIR;

    return wait_command();
}

uint64_t fsd_chdir(const char *dirname)
{
    spin_lock(&fs_op_lock);

    uint64_t name_phys = translate_addr(get_current_page_dir(), (uint64_t)dirname);
    uint64_t name_virt = USER_SPACE_BUFFER_MAPPING_OFFSET + name_phys;
    page_map_range_to(fsd_pgdir, name_virt, name_phys, strlen(dirname), USER_PTE_FLAGS);
    if (name_phys == 0)
    {
        return (uint64_t)-EINVAL;
    }
    fsd_user_command_addr->a = name_virt;
    fsd_user_command_addr->d = current_task->task_id;
    fsd_user_command_addr->cmd = FSD_CHDIR;

    return wait_command();
}

uint64_t fsd_getcwd(char *cwd)
{
    spin_lock(&fs_op_lock);

    uint64_t cwd_phys = translate_addr(get_current_page_dir(), (uint64_t)cwd);
    uint64_t cwd_virt = USER_SPACE_BUFFER_MAPPING_OFFSET + cwd_phys;
    page_map_range_to(fsd_pgdir, cwd_virt, cwd_phys, CWD_MAX_LEN, USER_PTE_FLAGS);
    if (cwd_virt == 0)
    {
        return (uint64_t)-EINVAL;
    }
    fsd_user_command_addr->a = cwd_virt;
    fsd_user_command_addr->d = current_task->task_id;
    fsd_user_command_addr->cmd = FSD_GETCWD;

    return wait_command();
}

uint64_t fsd_close(uint64_t fd)
{
    spin_lock(&fs_op_lock);

    fsd_user_command_addr->a = fd;
    fsd_user_command_addr->d = current_task->task_id;
    fsd_user_command_addr->cmd = FSD_CLOSE;

    return wait_command();
}
