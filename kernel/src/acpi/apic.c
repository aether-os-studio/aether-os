#include <kprint.h>
#include <acpi/acpi.h>
#include <mm/hhdm.h>
#include <mm/page.h>
#include <irq/irq.h>

spinlock_t apu_startup_lock;

bool x2apic_mode;
uint64_t lapic_address;
uint64_t ioapic_address;

__attribute__((used, section(".limine_requests"))) static volatile struct limine_mp_request mp_request = {
    .id = LIMINE_MP_REQUEST,
    .revision = 0,
};

void disable_pic()
{
    io_out8(0x21, 0xff);
    io_out8(0xa1, 0xff);

    io_out8(0x20, 0x20);
    io_out8(0xa0, 0x20);

    kdebug("8259A Masked.");

    io_out8(0x22, 0x70);
    io_out8(0x23, 0x01);
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

void lapic_write(uint32_t reg, uint32_t value)
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

void lapic_timer_stop();

void local_apic_init(bool is_print)
{
    x2apic_mode = ((mp_request.flags & LIMINE_MP_X2APIC) != 0);

    uint64_t value = rdmsr(0x1b);
    value |= (1UL << 11);
    if (x2apic_mode)
        value |= (1UL << 10);
    wrmsr(0x1b, value);

    lapic_timer_stop();

    lapic_write(LAPIC_REG_SPURIOUS, 0xff | (1 << 8));
    lapic_write(LAPIC_REG_TIMER_DIV, 11);
    lapic_write(LAPIC_REG_TIMER, APIC_TIMER_INTERRUPT_VECTOR);

    uint64_t b = nanoTime();
    lapic_write(LAPIC_REG_TIMER_INITCNT, ~((uint32_t)0));
    for (;;)
        if (nanoTime() - b >= 10000000)
            break;
    uint64_t lapic_timer = (~(uint32_t)0) - lapic_read(LAPIC_REG_TIMER_CURCNT);
    calibrated_timer_initial = (uint64_t)((uint64_t)(lapic_timer * 1000) / 250);
    if (is_print)
    {
        kinfo("Calibrated LAPIC timer: %d ticks per second.", calibrated_timer_initial);
    }
    lapic_write(LAPIC_REG_TIMER, lapic_read(LAPIC_REG_TIMER) | (1 << 17));
    lapic_write(LAPIC_REG_TIMER_INITCNT, calibrated_timer_initial);
    if (is_print)
    {
        kinfo("Setup local %s.", x2apic_mode ? "x2APIC" : "APIC");
    }
}

void local_apic_ap_init()
{
    uint64_t value = rdmsr(0x1b);
    value |= (1 << 11);
    if (x2apic_mode)
        value |= (1 << 10);
    wrmsr(0x1b, value);

    lapic_timer_stop();

    lapic_write(LAPIC_REG_SPURIOUS, 0xff | (1 << 8));
    lapic_write(LAPIC_REG_TIMER_DIV, 11);
    lapic_write(LAPIC_REG_TIMER, APIC_TIMER_INTERRUPT_VECTOR);

    lapic_write(LAPIC_REG_TIMER, lapic_read(LAPIC_REG_TIMER) | (1 << 17));
    lapic_write(LAPIC_REG_TIMER_INITCNT, calibrated_timer_initial);
}

void io_apic_init()
{
    page_map_range_to(get_kernel_page_dir(), phys_to_virt(ioapic_address), ioapic_address, PAGE_SIZE, KERNEL_PTE_FLAGS);
    ioapic_address = (uint64_t)phys_to_virt(ioapic_address);

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

void send_eoi(uint32_t irq)
{
    lapic_write(0xb0, 0);
    *(uint32_t *)(ioapic_address + 0x40) = irq;
}

void lapic_timer_stop()
{
    lapic_write(LAPIC_REG_TIMER_INITCNT, 0);
    lapic_write(LAPIC_REG_TIMER, (1 << 16));
}

void apic_setup(MADT *madt)
{
    lapic_address = phys_to_virt((uint64_t)madt->local_apic_address);
    page_map_range_to(get_kernel_page_dir(), lapic_address, madt->local_apic_address, PAGE_SIZE, KERNEL_PTE_FLAGS);

    kinfo("Setup Local apic: %#018lx.", lapic_address);

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
            break;
        }
        current += (uint64_t)header->length;
    }

    disable_pic();
    local_apic_init(true);
    io_apic_init();
}

#include <irq/trap.h>
#include <syscall/syscall.h>
#include <task/fsgsbase.h>
#include <task/task.h>

extern void sse_init();

extern bool task_initialized;

void ap_entry(struct limine_mp_info *cpu)
{
    cli();

    kinfo("APU %d starting", cpu->processor_id);

    uint64_t cr3 = (uint64_t)get_kernel_page_dir()->table;
    __asm__ __volatile__("movq %0, %%cr3" ::"r"(cr3) : "memory");

    sse_init();

    gdtidt_setup();

    syscall_init();

    local_apic_ap_init();

    tss_init();
    fsgsbase_init();

    spin_unlock(&apu_startup_lock);

    while (!task_initialized)
    {
        __asm__ __volatile__("pause");
    }

    task_switch_to(NULL, NULL, idle_tasks[cpu->processor_id]);

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

    kerror("Cannot get cpu id, lapic id = %d", lapic_id);

    return 0;
}

void apu_startup(struct limine_mp_response *smp_response)
{
    cpu_count = smp_response->cpu_count;

    for (uint64_t i = 0; i < smp_response->cpu_count; i++)
    {
        struct limine_mp_info *cpu = smp_response->cpus[i];
        cpuid_to_lapicid[cpu->processor_id] = cpu->lapic_id;

        if (cpu->lapic_id == smp_response->bsp_lapic_id)
            continue;

        spin_lock(&apu_startup_lock);

        cpu->goto_address = ap_entry;
    }
}

void smp_init()
{
    spin_init(&apu_startup_lock);

    apu_startup(mp_request.response);
}
