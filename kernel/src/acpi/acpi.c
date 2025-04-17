#include <kprint.h>
#include <mm/hhdm.h>
#include <acpi/acpi.h>
#include <mm/page.h>

#define load_table(name, func)                          \
    do                                                  \
    {                                                   \
        void *name = find_table(#name);                 \
        if (name == NULL)                               \
        {                                               \
            kwarn("Cannot find acpi " #name " table."); \
            return;                                     \
        }                                               \
        else                                            \
        {                                               \
            func(name);                                 \
        }                                               \
    } while (0);

uint64_t rsdp_paddr;

XSDT *xsdt;

__attribute__((used, section(".limine_requests"))) static volatile struct limine_rsdp_request rsdp_request = {.id = LIMINE_RSDP_REQUEST, .revision = 0, .response = NULL};

void *find_table(const char *name)
{
    uint64_t entry_count = (xsdt->h.Length - 32) / 8;
    uint64_t *t = (uint64_t *)((char *)xsdt + offsetof(XSDT, PointerToOtherSDT));
    for (uint64_t i = 0; i < entry_count; i++)
    {
        uint64_t phys = (uint64_t)(*(t + i));
        uint64_t ptr = phys_to_virt(phys);
        page_map_range_to(get_kernel_page_dir(), ptr, phys, PAGE_SIZE, KERNEL_PTE_FLAGS);
        uint8_t signa[5] = {0};
        memcpy(signa, ((struct ACPISDTHeader *)ptr)->Signature, 4);
        if (memcmp(signa, name, 4) == 0)
        {
            return (void *)ptr;
        }
    }
    return NULL;
}

void acpi_init()
{
    struct limine_rsdp_response *response = rsdp_request.response;

    rsdp_paddr = response->address;

    RSDP *rsdp = (RSDP *)rsdp_paddr;
    if (rsdp == NULL)
    {
        kwarn("Cannot find acpi RSDP table.");
        return;
    }
    rsdp = phys_to_virt(rsdp);
    page_map_range_to(get_kernel_page_dir(), (uint64_t)rsdp, rsdp_paddr, PAGE_SIZE, KERNEL_PTE_FLAGS);

    uint64_t xsdt_paddr = rsdp->xsdt_address;

    xsdt = (XSDT *)xsdt_paddr;
    if (xsdt == NULL)
    {
        kwarn("Cannot find acpi XSDT table.");
        return;
    }
    xsdt = phys_to_virt(xsdt);
    page_map_range_to(get_kernel_page_dir(), (uint64_t)xsdt, xsdt_paddr, PAGE_SIZE, KERNEL_PTE_FLAGS);

    load_table(HPET, hpet_setup);
    load_table(APIC, apic_setup);
    // load_table(MCFG, pcie_setup);
    // load_table(FACP, facp_setup);
}