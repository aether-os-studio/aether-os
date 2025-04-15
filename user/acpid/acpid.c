#include <libdaemon.h>
#include <stdio.h>
#include "acpi.h"

user_scheme_t acpid_scheme;

RSDP *rsdp;
XSDT *xsdt;

const char *read_target;

void *find_table(const char *name)
{
    uint64_t entry_count = (xsdt->h.Length - 32) / 8;
    uint64_t *t = (uint64_t *)((char *)xsdt + offsetof(XSDT, PointerToOtherSDT));
    for (uint64_t i = 0; i < entry_count; i++)
    {
        uint64_t phys = (uint64_t)(*(t + i));
        uint64_t ptr = physmap(phys, sizeof(struct ACPISDTHeader), PROT_READ | PROT_WRITE);
        uint8_t signa[5] = {0};
        memcpy(signa, ((struct ACPISDTHeader *)ptr)->Signature, 4);
        if (memcmp(signa, (char *)name, 4) == 0)
        {
            return (void *)ptr;
        }
    }
    return NULL;
}

uint64_t acpid_read(uint64_t buf, uint64_t len, uint64_t offset, char *target_name)
{
    void *ptr = find_table(target_name);
    if (!ptr)
        return 0;

    memcpy((void *)buf, ptr, len);
    return len;
}

uint64_t acpid_write(uint64_t buf, uint64_t len, uint64_t offset, char *target_name)
{
    return (uint64_t)-1;
}

uint64_t acpid_ioctl(uint64_t cmd, uint64_t arg, uint64_t offset, char *target_name)
{
    switch (cmd)
    {
    case SCHEME_IOCTL_GETSIZE:
        struct ACPISDTHeader *ptr = (struct ACPISDTHeader *)find_table(target_name);
        if (ptr)
            return ptr->Length;
    default:
        break;
    }

    return 0;
}

uint64_t acpid_daemon(daemon_t *daemon)
{
    printf("acpi daemon is running\n");

    bootstrap_info_t info;
    get_bootstrap_info(&info);

    uint64_t virt = physmap(info.rsdp_phys_address, sizeof(RSDP), PROT_READ | PROT_WRITE);
    printf("RSDP virt address: %#018lx\n", virt);

    rsdp = (RSDP *)virt;
    printf("RSDT address: %#018lx\n", rsdp->rsdt_address);
    printf("XSDT address: %#018lx\n", rsdp->xsdt_address);
    if (rsdp->xsdt_address == 0)
    {
        return -EINVAL;
    }
    xsdt = (XSDT *)physmap(rsdp->xsdt_address, sizeof(XSDT), PROT_READ | PROT_WRITE);

    init_scheme(&acpid_scheme);

    scheme_create("/scheme/acpid", &acpid_scheme);

    finish_daemon(daemon);

    while (1)
    {
        if (acpid_scheme.command.cmd != 0)
        {
            uint64_t result = 0;
            switch (acpid_scheme.command.cmd)
            {
            case SCHEME_COMMAND_READ:
                result = acpid_read(acpid_scheme.command.a, acpid_scheme.command.b, acpid_scheme.command.c, (char *)acpid_scheme.command.d);
                break;
            case SCHEME_COMMAND_WRITE:
                result = acpid_write(acpid_scheme.command.a, acpid_scheme.command.b, acpid_scheme.command.c, (char *)acpid_scheme.command.d);
                break;
            case SCHEME_COMMAND_IOCTL:
                result = acpid_ioctl(acpid_scheme.command.a, acpid_scheme.command.b, acpid_scheme.command.c, (char *)acpid_scheme.command.d);
                break;
            default:
                printf("unknown command\n");
                break;
            }

            acpid_scheme.command.a = result;
            acpid_scheme.command.cmd = 0;
        }

        __asm__ __volatile__("pause");
    }
}

int main()
{
    return start_daemon(acpid_daemon);
}
