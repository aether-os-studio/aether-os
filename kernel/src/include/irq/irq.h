#pragma once

#include <klibc.h>

typedef struct hardware_intr_type
{
    void (*enable)(uint8_t irq_num);
    void (*disable)(uint8_t irq_num);

    void (*install)(uint8_t irq_num, uint32_t arg);
} hardware_intr_controller;

#define IRQ_NAME_LEN 32

typedef struct
{
    hardware_intr_controller *controller;
    char irq_name[IRQ_NAME_LEN];
    uint64_t parameter;
    void (*handler)(uint8_t irq_num, uint64_t parameter, struct pt_regs *regs);
    uint64_t flags;
} irq_desc_t;

void generic_interrupt_table_init();
