#include <kprint.h>
#include <acpi/acpi.h>
#include <mm/hhdm.h>
#include <mm/page.h>

HpetInfo *hpet_addr;
static uint32_t hpetPeriod = 0;

void usleep(uint64_t nano)
{
    uint64_t targetTime = nanoTime();
    uint64_t after = 0;
    while (1)
    {
        uint64_t n = nanoTime();
        if (n < targetTime)
        {
            after += 0xffffffff - targetTime + n;
            targetTime = n;
        }
        else
        {
            after += n - targetTime;
            targetTime = n;
        }
        if (after >= nano)
        {
            return;
        }
    }
}

uint64_t nanoTime()
{
    if (hpet_addr == NULL)
        return 0;
    uint64_t mcv = hpet_addr->mainCounterValue;
    return mcv * hpetPeriod;
}

void hpet_setup(Hpet *hpet)
{
    hpet_addr = (HpetInfo *)phys_to_virt(hpet->base_address.address);
    page_map_range_to(get_kernel_page_dir(), (uint64_t)hpet_addr, hpet->base_address.address, PAGE_SIZE, KERNEL_PTE_FLAGS);
    uint32_t counterClockPeriod = hpet_addr->generalCapabilities >> 32;
    hpetPeriod = counterClockPeriod / 1000000;
    hpet_addr->generalConfiguration |= 1;
    *(volatile uint64_t *)((uint64_t)hpet_addr + 0xf0) = 0;
    kinfo("Setup acpi hpet table (nano_time: %#ld).", (uint64_t)nanoTime());
}
