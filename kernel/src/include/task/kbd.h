#pragma once

#include <acpi/acpi.h>
#include <irq/irq.h>

#define KB_BUF_SIZE 64

struct keyboard_buf
{
    uint8_t *p_head;
    uint8_t *p_tail;
    int32_t count;
    bool ctrl;
    bool shift;
    bool alt;
    bool caps;
    uint8_t buf[KB_BUF_SIZE];
};

extern struct keyboard_buf kb_fifo;

void parse_scan_code(uint8_t x);
uint8_t get_keyboard_input();

void kbd_init();
