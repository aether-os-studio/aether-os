#include <scheme/scheme.h>
#include <mm/hhdm.h>
#include <mm/page.h>
#include <irq/irq.h>
#include <kprint.h>

bool taskid_to_hasop[MAX_TASK_NUM];
scheme_t taskid_to_user_schemes[MAX_TASK_NUM];

void scheme_init()
{
    memset(taskid_to_hasop, 0, sizeof(taskid_to_hasop));
    memset(taskid_to_user_schemes, 0, sizeof(taskid_to_user_schemes));
}

void scheme_create(const char *name, uint64_t ptr)
{
    scheme_t *scheme = &taskid_to_user_schemes[current_task->task_id];
    strncpy(scheme->name, name, SCHEME_NAME_MAX);
    scheme->task = current_task;
    scheme->user_scheme = (void *)translate_addr(get_current_page_dir(), ptr);
}

scheme_t *scheme_open(const char *name)
{
    for (uint64_t i = 0; i < MAX_TASK_NUM; i++)
    {
        if (taskid_to_user_schemes[i].name[0] == '\0')
            continue;

        if (!strcmp(name, taskid_to_user_schemes[i].name))
        {
            return &taskid_to_user_schemes[i];
        }
    }

    return NULL;
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

    command->cmd = cmd;
    command->a = buffer;
    command->b = len;

    while (command->cmd)
    {
        sti();

        // Wait for the command to be processed
        __asm__ __volatile__("int %0\n\t" ::"i"(APIC_TIMER_INTERRUPT_VECTOR));
    }

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