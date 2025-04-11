#include <kprint.h>
#include <acpi/acpi.h>
#include <mm/hhdm.h>
#include <mm/page.h>
#include <irq/irq.h>

bool x2apic_mode;
uint64_t lapic_address;
uint64_t ioapic_address;

__attribute__((used, section(".limine_requests"))) static volatile struct limine_mp_request mp_request = {
    .id = LIMINE_MP_REQUEST,
    .revision = 0,
    .response = NULL,
    .flags = 1U,
};

void disable_pic()
{
    io_out8(0x21, 0xff);
    io_out8(0xa1, 0xff);
}

static void ioapic_write(uint32_t reg, uint32_t value)
{
    *(uint32_t *)(ioapic_address) = reg;
    *(uint32_t *)((uint64_t)ioapic_address + 0x10) = value;
}

static uint32_t ioapic_read(uint32_t reg)
{
    *(uint32_t *)(ioapic_address) = reg;
    return *(uint32_t *)((uint64_t)ioapic_address + 0x10);
}

void ioapic_add(uint8_t vector, uint32_t irq)
{
    uint32_t ioredtbl = (uint32_t)(0x10 + (uint32_t)(irq * 2));
    uint64_t redirect = (uint64_t)vector;
    redirect |= lapic_id() << 56;
    ioapic_write(ioredtbl, (uint32_t)redirect);
    ioapic_write(ioredtbl + 1, (uint32_t)(redirect >> 32));
}

static void lapic_write(uint32_t reg, uint32_t value)
{
    if (x2apic_mode)
    {
        wrmsr(0x800 + (reg >> 4), value);
        return;
    }
    *(uint32_t *)((uint64_t)lapic_address + reg) = value;
}

uint32_t lapic_read(uint32_t reg)
{
    if (x2apic_mode)
    {
        return rdmsr(0x800 + (reg >> 4));
    }
    return *(uint32_t *)((uint64_t)lapic_address + reg);
}

uint64_t lapic_id()
{
    uint32_t phy_id = lapic_read(LAPIC_REG_ID);
    return x2apic_mode ? phy_id : (phy_id >> 24);
}

uint64_t calibrated_timer_initial;

void local_apic_init(bool is_print)
{
    x2apic_mode = (mp_request.response->flags & 1U) != 0;

    lapic_write(LAPIC_REG_TIMER, APIC_TIMER_INTERRUPT_VECTOR);
    lapic_write(LAPIC_REG_SPURIOUS, 0xff | 1 << 8);
    lapic_write(LAPIC_REG_TIMER_DIV, 11);

    uint64_t b = nanoTime();
    lapic_write(LAPIC_REG_TIMER_INITCNT, ~((uint32_t)0));
    for (;;)
        if (nanoTime() - b >= 1000000)
            break;
    uint64_t lapic_timer = (~(uint32_t)0) - lapic_read(LAPIC_REG_TIMER_CURCNT);
    calibrated_timer_initial = (uint64_t)((uint64_t)(lapic_timer * 1000) / 250);
    if (is_print)
    {
        kinfo("Calibrated LAPIC timer: %d ticks per second.", calibrated_timer_initial);
    }
    lapic_write(LAPIC_REG_TIMER, lapic_read(LAPIC_REG_TIMER) | 1 << 17);
    lapic_write(LAPIC_REG_TIMER_INITCNT, calibrated_timer_initial);
    if (is_print)
    {
        kinfo("Setup local %s.", x2apic_mode ? "x2APIC" : "APIC");
    }
}

void local_apic_ap_init()
{
    lapic_write(LAPIC_REG_TIMER, APIC_TIMER_INTERRUPT_VECTOR);
    lapic_write(LAPIC_REG_SPURIOUS, 0xff | 1 << 8);
    lapic_write(LAPIC_REG_TIMER_DIV, 11);

    lapic_write(LAPIC_REG_TIMER, lapic_read(LAPIC_REG_TIMER) | 1 << 17);
    lapic_write(LAPIC_REG_TIMER_INITCNT, calibrated_timer_initial);
}

void io_apic_init()
{
    page_map_range_to(get_kernel_page_dir(), phys_to_virt(ioapic_address), ioapic_address, PAGE_SIZE, KERNEL_PTE_FLAGS);
    ioapic_address = (uint64_t)phys_to_virt(ioapic_address);
    // ioapic_add((uint8_t)APIC_TIMER_INTERRUPT_VECTOR, 0);
    // ioapic_add((uint8_t)keyboard, 1);
    // ioapic_add((uint8_t)mouse, 12);
    // ioapic_add((uint8_t)ide_primary, 14);
    // ioapic_add((uint8_t)ide_secondary, 15);

    kinfo("Setup I/O apic: %#018lx.", ioapic_address);
}

void ioapic_enable(uint8_t vector)
{
    uint64_t index = 0x10 + ((vector - 32) * 2);
    uint64_t value = (uint64_t)ioapic_read(index + 1) << 32 | (uint64_t)ioapic_read(index);
    value &= (~0x10000UL);
    ioapic_write(index, (uint32_t)(value & 0xFFFFFFFF));
    ioapic_write(index + 1, (uint32_t)(value >> 32));
}

void ioapic_disable(uint8_t vector)
{
    uint64_t index = 0x10 + ((vector - 32) * 2);
    uint64_t value = (uint64_t)ioapic_read(index + 1) << 32 | (uint64_t)ioapic_read(index);
    value |= 0x10000UL;
    ioapic_write(index, (uint32_t)(value & 0xFFFFFFFF));
    ioapic_write(index + 1, (uint32_t)(value >> 32));
}

void send_eoi()
{
    lapic_write(0xb0, 0);
}

void lapic_timer_stop()
{
    lapic_write(LAPIC_REG_TIMER_INITCNT, 0);
    lapic_write(LAPIC_REG_TIMER, (1 << 16));
}

void send_ipi(uint32_t apic_id, uint32_t command)
{
    if (x2apic_mode)
    {
        lapic_write(APIC_ICR_LOW, (((uint64_t)apic_id) << 32) | command);
    }
    else
    {
        lapic_write(APIC_ICR_HIGH, apic_id << 24);
        lapic_write(APIC_ICR_LOW, command);
    }
}

void apic_setup(MADT *madt)
{
    lapic_address = phys_to_virt((uint64_t)madt->local_apic_address);
    page_map_range_to(get_kernel_page_dir(), lapic_address, madt->local_apic_address, PAGE_SIZE, KERNEL_PTE_FLAGS);

    uint64_t current = 0;
    for (;;)
    {
        if (current + ((uint32_t)sizeof(MADT) - 1) >= madt->h.Length)
        {
            break;
        }
        MadtHeader *header = (MadtHeader *)((uint64_t)(&madt->entries) + current);
        if (header->entry_type == MADT_APIC_IO)
        {
            MadtIOApic *ioapic = (MadtIOApic *)((uint64_t)(&madt->entries) + current);
            ioapic_address = ioapic->address;
        }
        current += (uint64_t)header->length;
    }
    disable_pic();
    local_apic_init(true);
    io_apic_init();
}

#include <irq/trap.h>
#include <task/fsgsbase.h>
#include <task/task.h>

extern void sse_init();

extern bool task_initialized;

void ap_entry(struct limine_mp_info *cpu)
{
    uint64_t cr3 = (uint64_t)get_kernel_page_dir()->table;
    __asm__ __volatile__("movq %0, %%cr3" ::"r"(cr3) : "memory");

    sse_init();

    fsgsbase_init();

    gdtidt_setup();

    while (!task_initialized)
    {
    }

    local_apic_ap_init();

    write_gsbase((uint64_t)idle_task);

    while (1)
    {
        sti();
        hlt();
    }
}

uint64_t cpu_count;

uint32_t cpuid_to_lapicid[MAX_CPU_NUM];

uint32_t get_cpuid_by_lapic_id(uint32_t lapic_id)
{
    for (uint32_t cpu_id = 0; cpu_id < cpu_count; cpu_id++)
    {
        if (cpuid_to_lapicid[cpu_id] == lapic_id)
        {
            return cpu_id;
        }
    }

    return 0;
}

void apu_startup(struct limine_mp_response *smp_response)
{
    cpu_count = smp_response->cpu_count;

    for (uint64_t i = 0; i < smp_response->cpu_count; i++)
    {
        struct limine_mp_info *cpu = smp_response->cpus[i];
        cpuid_to_lapicid[i] = cpu->lapic_id;
        if (cpu->lapic_id == smp_response->bsp_lapic_id)
            continue;

        cpu->goto_address = ap_entry;
    }
}

void smp_init()
{
    apu_startup(mp_request.response);
}
