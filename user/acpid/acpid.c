#include <libdaemon.h>
#include <stdio.h>
#include "acpi.h"

RSDP *rsdp;

uint64_t acpid_daemon(daemon_t *daemon)
{
    printf("acpid daemon is running\n");

    bootstrap_info_t info;
    get_bootstrap_info(&info);

    uint64_t virt = physmap(info.rsdp_phys_address, sizeof(RSDP), PROT_READ | PROT_WRITE);
    printf("RSDP virt address: %#018lx\n", virt);

    rsdp = (RSDP *)virt;
    printf("RSDT address: %#018lx\n", rsdp->rsdt_address);
    printf("XSDT address: %#018lx\n", rsdp->xsdt_address);

    finish_daemon(daemon);

    return 0;
}

int main()
{
    return start_daemon(acpid_daemon);
}
