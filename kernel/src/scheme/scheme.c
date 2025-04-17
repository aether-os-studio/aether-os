#include <scheme/scheme.h>
#include <mm/hhdm.h>
#include <mm/page.h>
#include <irq/irq.h>
#include <task/task.h>
#include <kprint.h>
#include <syscall/syscall.h>

scheme_t taskid_to_user_schemes[MAX_TASK_NUM];

void scheme_init()
{
    memset(taskid_to_user_schemes, 0, sizeof(taskid_to_user_schemes));
}

void scheme_create(const char *name, uint64_t ptr)
{
    scheme_t *scheme = &taskid_to_user_schemes[current_task->task_id];
    strncpy(scheme->name, name, SCHEME_NAME_MAX);
    scheme->task = current_task;
    scheme->user_scheme = (void *)translate_addr(get_current_page_dir(), ptr);
    scheme->offset = 0;
    user_scheme_t *uscheme = phys_to_virt((user_scheme_t *)scheme->user_scheme);
    user_scheme_command_t *command = &uscheme->command;

    uint64_t command_d = translate_addr(get_current_page_dir(), command->d);
    if (command_d == 0)
    {
        kerror("Invalid user scheme");
        return;
    }
    scheme->command_d = command_d;
}

scheme_t *scheme_open(const char *name)
{
    for (uint64_t i = 0; i < MAX_TASK_NUM; i++)
    {
        if (taskid_to_user_schemes[i].name[0] == '\0')
            continue;

        if (taskid_to_user_schemes[i].target_name[0] != '\0')
            continue;

        if (!strncmp(name, taskid_to_user_schemes[i].name, strlen(taskid_to_user_schemes[i].name)))
        {
            uint64_t idx = strlen(taskid_to_user_schemes[i].name) + 1;
            scheme_t *scheme = &taskid_to_user_schemes[i];
            strncpy(scheme->target_name, name + idx, SCHEME_NAME_MAX);
            return scheme;
        }
    }

    return NULL;
}

void scheme_close(scheme_t *scheme)
{
    scheme->target_name[0] = '\0';
}

uint64_t scheme_transfer(scheme_t *scheme, uint64_t cmd, uint64_t buffer, uint64_t len)
{
    user_scheme_t *uscheme = scheme->user_scheme;
    if (uscheme == NULL)
    {
        return (uint64_t)-EINVAL;
    }

    uscheme = phys_to_virt(uscheme);
    user_scheme_command_t *command = &uscheme->command;

    if (cmd == SCHEME_COMMAND_READ || cmd == SCHEME_COMMAND_WRITE || cmd == SCHEME_COMMAND_READDIR)
    {
        uint64_t buffer_phys = translate_addr(get_current_page_dir(), buffer);
        if (buffer_phys == 0)
        {
            kerror("Invalid buffer address");
            return (uint64_t)-EINVAL;
        }

        uint64_t buffer_virt = USER_SPACE_BUFFER_MAPPING_OFFSET + buffer_phys;
        page_map_range_to(scheme->task->pgdir, buffer_virt, buffer_phys, len + PAGE_SIZE, USER_PTE_FLAGS);

        command->a = buffer_virt;
    }
    else
    {
        command->a = buffer;
    }
    command->b = len;
    command->c = scheme->offset;

    memset(phys_to_virt((char *)scheme->command_d), 0, SCHEME_NAME_MAX);
    strncpy(phys_to_virt((char *)scheme->command_d), scheme->target_name, SCHEME_NAME_MAX);

    command->cmd = cmd;

    while (command->cmd)
    {
        sti();

        // Wait for the command to be processed
        sys_yield();
    }

    cli();

    return command->a;
}

uint64_t scheme_read(scheme_t *scheme, uint64_t buffer, uint64_t len)
{
    return scheme_transfer(scheme, SCHEME_COMMAND_READ, buffer, len);
}

uint64_t scheme_write(scheme_t *scheme, uint64_t buffer, uint64_t len)
{
    return scheme_transfer(scheme, SCHEME_COMMAND_WRITE, buffer, len);
}

uint64_t scheme_ioctl(scheme_t *scheme, uint64_t buffer, uint64_t len)
{
    return scheme_transfer(scheme, SCHEME_COMMAND_IOCTL, buffer, len);
}

uint64_t scheme_readdir(scheme_t *scheme, uint64_t buffer, uint64_t len)
{
    return scheme_transfer(scheme, SCHEME_COMMAND_READDIR, buffer, len);
}
