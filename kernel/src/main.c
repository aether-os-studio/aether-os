#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <limine.h>

__attribute__((used, section(".limine_requests"))) static volatile LIMINE_BASE_REVISION(3);
// Finally, define the start and end markers for the Limine requests.
// These can also be moved anywhere, to any .c file, as seen fit.

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

// The following will be our kernel's entry point.
// If renaming kmain() to something else, make sure to change the
// linker script accordingly.
void kmain(void)
{
    // Ensure the bootloader actually understands our base revision (see spec).
    if (LIMINE_BASE_REVISION_SUPPORTED == false)
    {
        hcf();
    }

    sse_init();

    printk_init(8, 16);

    hhdm_init();
    frame_init();

    // We're done, just hang...
    hcf();
}
