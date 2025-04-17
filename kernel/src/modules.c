#include <klibc.h>
#include <task/execve.h>
#include <kprint.h>
#include <task/task.h>

__attribute__((used, section(".limine_requests"))) static volatile struct limine_module_request module_request = {
    .id = LIMINE_MODULE_REQUEST,
};

void init_module()
{
    struct limine_module_response *response = module_request.response;
    if (response)
    {
        for (uint64_t i = 0; i < response->module_count; i++)
        {
            struct limine_file *module = response->modules[i];
            if (!strcmp(module->path, "/usr/bin/initd.exec"))
            {
                load_module(module, NULL, NULL);
            }
        }
    }
}

void sys_load_module(const char *name, char **argv, char **envp)
{
    struct limine_module_response *response = module_request.response;
    if (response)
    {
        for (uint64_t i = 0; i < response->module_count; i++)
        {
            struct limine_file *module = response->modules[i];
            if (!strcmp(module->path, name))
            {
                strcpy(current_task->name, name);
                load_module(module, argv, envp);
            }
        }

        kerror("No module %s found", name);
    }
}
