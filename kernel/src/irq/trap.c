#include <irq/gate.h>
#include <irq/trap.h>
#include <irq/irq.h>
#include <task/task.h>
#include <kprint.h>

void irq_init()
{
    gdtidt_setup();

    set_trap_gate(0, 0, divide_error);
    set_trap_gate(1, 0, debug);
    set_intr_gate(2, 0, nmi);
    set_system_trap_gate(3, 0, int3);
    set_system_trap_gate(4, 0, overflow);
    set_system_trap_gate(5, 0, bounds);
    set_trap_gate(6, 0, undefined_opcode);
    set_trap_gate(7, 0, dev_not_avaliable);
    set_trap_gate(8, 0, double_fault);
    set_trap_gate(9, 0, coprocessor_segment_overrun);
    set_trap_gate(10, 0, invalid_TSS);
    set_trap_gate(11, 0, segment_not_exists);
    set_trap_gate(12, 0, stack_segment_fault);
    set_trap_gate(13, 0, general_protection);
    set_trap_gate(14, 0, page_fault);
    // 中断号15由Intel保留，不能使用
    set_trap_gate(16, 0, x87_FPU_error);
    set_trap_gate(17, 0, alignment_check);
    set_trap_gate(18, 0, machine_check);
    set_trap_gate(19, 0, SIMD_exception);
    set_trap_gate(20, 0, virtualization_exception);

    generic_interrupt_table_init();
}

void dump_regs(struct pt_regs *regs)
{
    // printk("CPU ID = %d\n", current_cpu_id);

    printk("RAX = %#018lx, RBX = %#018lx\n", regs->rax, regs->rbx);
    printk("RCX = %#018lx, RDX = %#018lx\n", regs->rcx, regs->rdx);
    printk("RDI = %#018lx, RSI = %#018lx\n", regs->rdi, regs->rsi);
    printk("RSP = %#018lx, RBP = %#018lx\n", regs->rsp, regs->rbp);
    printk(" R8 = %#018lx,  R9 = %#018lx\n", regs->r8, regs->r9);
    printk("R10 = %#018lx, R11 = %#018lx\n", regs->r10, regs->r11);
    printk("R12 = %#018lx, R13 = %#018lx\n", regs->r12, regs->r13);
    printk("R14 = %#018lx, R15 = %#018lx\n", regs->r14, regs->r15);
}

// 0 #DE 除法错误
void do_divide_error(struct pt_regs *regs, uint64_t error_code)
{
    kerror("do_divide_error(0)");

    dump_regs(regs);

    while (1)
        hlt();
}

// 1 #DB 调试异常
void do_debug(struct pt_regs *regs, uint64_t error_code)
{
    printk("[ ");
    printk_color(RED, BLACK, "ERROR / TRAP");
    printk(" ] do_debug(1),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 2 不可屏蔽中断
void do_nmi(struct pt_regs *regs, uint64_t error_code)
{
    printk("[ ");
    printk_color(BLUE, BLACK, "INT");
    printk(" ] do_nmi(2),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 3 #BP 断点异常
void do_int3(struct pt_regs *regs, uint64_t error_code)
{
    printk("[ ");
    printk_color(YELLOW, BLACK, "TRAP");
    printk(" ] do_int3(3),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    return;
}

// 4 #OF 溢出异常
void do_overflow(struct pt_regs *regs, uint64_t error_code)
{
    printk("[ ");
    printk_color(YELLOW, BLACK, "TRAP");
    printk(" ] do_overflow(4),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 5 #BR 越界异常
void do_bounds(struct pt_regs *regs, uint64_t error_code)
{
    kerror("do_bounds(5),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 6 #UD 无效/未定义的机器码
void do_undefined_opcode(struct pt_regs *regs, uint64_t error_code)
{
    kerror("do_undefined_opcode(6),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 7 #NM 设备异常（FPU不存在）
void do_dev_not_avaliable(struct pt_regs *regs, uint64_t error_code)
{
    kerror("do_dev_not_avaliable(7),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 8 #DF 双重错误
void do_double_fault(struct pt_regs *regs, uint64_t error_code)
{
    printk("[ ");
    printk_color(RED, BLACK, "Terminate");
    printk(" ] do_double_fault(8),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 9 协处理器越界（保留）
void do_coprocessor_segment_overrun(struct pt_regs *regs, uint64_t error_code)
{
    kerror("do_coprocessor_segment_overrun(9),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 10 #TS 无效的TSS段
void do_invalid_TSS(struct pt_regs *regs, uint64_t error_code)
{
    printk("[");
    printk_color(RED, BLACK, "ERROR");
    printk("] do_invalid_TSS(10),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    printk_color(YELLOW, BLACK, "Information:\n");
    // 解析错误码
    if (error_code & 0x01)
        printk("The exception occurred during delivery of an event external to the program.\n");

    if (error_code & 0x02)
        printk("Refers to a descriptor in the IDT.\n");
    else
    {
        if (error_code & 0x04)
            printk("Refers to a descriptor in the current LDT.\n");
        else
            printk("Refers to a descriptor in the GDT.\n");
    }

    printk("Segment Selector Index:%10x\n", error_code & 0xfff8);

    printk("\n");

    dump_regs(regs);

    while (1)
        hlt();
}

// 11 #NP 段不存在
void do_segment_not_exists(struct pt_regs *regs, uint64_t error_code)
{
    kerror("do_segment_not_exists(11),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 12 #SS SS段错误
void do_stack_segment_fault(struct pt_regs *regs, uint64_t error_code)
{
    kerror("do_stack_segment_fault(12),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 13 #GP 通用保护性异常
void do_general_protection(struct pt_regs *regs, uint64_t error_code)
{
    kerror("do_general_protection(13),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);
    if (error_code & 0x01)
        printk_color(RED, BLACK, "The exception occurred during delivery of an event external to the program,such as an interrupt or an earlier exception.\n");

    if (error_code & 0x02)
        printk_color(RED, BLACK, "Refers to a gate descriptor in the IDT;\n");
    else
        printk_color(RED, BLACK, "Refers to a descriptor in the GDT or the current LDT;\n");

    if ((error_code & 0x02) == 0)
        if (error_code & 0x04)
            printk_color(RED, BLACK, "Refers to a segment or gate descriptor in the LDT;\n");
        else
            printk_color(RED, BLACK, "Refers to a descriptor in the current GDT;\n");

    printk_color(RED, BLACK, "Segment Selector Index:%#010x\n", error_code & 0xfff8);

    dump_regs(regs);

    while (1)
        hlt();
}

// 14 #PF 页故障
void do_page_fault(struct pt_regs *regs, uint64_t error_code)
{
    uint64_t cr2 = 0;
    // 先保存cr2寄存器的值，避免由于再次触发页故障而丢失值
    // cr2存储着触发异常的线性地址
    __asm__ __volatile__("movq %%cr2, %0"
                         : "=r"(cr2)::"memory");

    kerror("do_page_fault(14),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\tCR2:%#18lx\n", error_code, regs->rsp, regs->rip, cr2);

    printk_color(YELLOW, BLACK, "Information:\n");
    if (!(error_code & 0x01))
        printk("Page does not exist.\n");

    if (error_code & 0x02)
        printk("Fault occurred during operation: writing\n");
    else
        printk("Fault occurred during operation: reading\n");

    if (error_code & 0x04)
        printk("Fault in user level(3).\n");
    else
        printk("Fault in supervisor level(0,1,2).\n");

    if (error_code & 0x08)
        printk("Reserved bit caused the fault.\n");

    if (error_code & 0x10)
        printk("Fault occurred during fetching instruction.\n");

    dump_regs(regs);

    while (1)
        hlt();
}

// 15 Intel保留，请勿使用

// 16 #MF x87FPU错误
void do_x87_FPU_error(struct pt_regs *regs, uint64_t error_code)
{
    kerror("do_x87_FPU_error(16),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 17 #AC 对齐检测
void do_alignment_check(struct pt_regs *regs, uint64_t error_code)
{
    kerror("do_alignment_check(17),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 18 #MC 机器检测
void do_machine_check(struct pt_regs *regs, uint64_t error_code)
{
    kerror("do_machine_check(18),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 19 #XM SIMD浮点异常
void do_SIMD_exception(struct pt_regs *regs, uint64_t error_code)
{
    kerror("do_SIMD_exception(19),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}

// 20 #VE 虚拟化异常
void do_virtualization_exception(struct pt_regs *regs, uint64_t error_code)
{
    kerror("do_virtualization_exception(20),\tError Code:%#18lx,\tRSP:%#18lx,\tRIP:%#18lx\n", error_code, regs->rsp, regs->rip);

    dump_regs(regs);

    while (1)
        hlt();
}
