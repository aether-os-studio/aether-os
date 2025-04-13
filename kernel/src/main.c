#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <limine.h>

__attribute__((used, section(".limine_requests"))) static volatile LIMINE_BASE_REVISION(3);

__attribute__((used, section(".limine_requests_start"))) static volatile LIMINE_REQUESTS_START_MARKER;

__attribute__((used, section(".limine_requests_end"))) static volatile LIMINE_REQUESTS_END_MARKER;

static void hcf(void)
{
    for (;;)
    {
        asm("hlt");
    }
}

void sse_init()
{
    __asm__ __volatile__("movq %cr0, %rax\n\t"
                         "and $0xFFF3, %ax	\n\t" // clear coprocessor emulation CR0.EM and CR0.TS
                         "or $0x2, %ax\n\t"       // set coprocessor monitoring  CR0.MP
                         "movq %rax, %cr0\n\t"
                         "movq %cr4, %rax\n\t"
                         "or $(3 << 9), %ax\n\t" // set CR4.OSFXSR and CR4.OSXMMEXCPT at the same time
                         "movq %rax, %cr4\n\t");
}

#include <kprint.h>
#include <mm/hhdm.h>
#include <mm/frame.h>
#include <mm/page.h>
#include <mm/heap.h>
#include <acpi/acpi.h>
#include <irq/gate.h>
#include <irq/trap.h>
#include <syscall/syscall.h>
#include <scheme/scheme.h>
#include <task/fsgsbase.h>
#include <task/task.h>

void kmain(void)
{
    cli();

    if (LIMINE_BASE_REVISION_SUPPORTED == false)
    {
        hcf();
    }

    sse_init();

    printk_init(8, 16);

    hhdm_init();
    frame_init();
    page_init();
    heap_init();

    acpi_init();
    irq_init();

    smp_init();

    syscall_init();

    scheme_init();

    tss_init();
    fsgsbase_init();
    task_init();

    while (1)
    {
        sti();
        hlt();
    }
}
