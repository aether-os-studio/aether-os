#include <task/kbd.h>

enum special_key_code
{
    KEY_ESC = 128,
    KEY_BACKSPACE,
    KEY_TAB,
    KEY_ENTER,
    KEY_CAPS,
    KEY_SHIFT,
    KEY_CTRL,
    KEY_ALT,
    KEY_SPACE,
    KEY_F1,
    KEY_F2,
    KEY_F3,
    KEY_F4,
    KEY_F5,
    KEY_F6,
    KEY_F7,
    KEY_F8,
    KEY_F9,
    KEY_F10,
    KEY_NUML,
    KEY_SCROLL,
    KEY_F11,
    KEY_F12
};

uint8_t keyboard_code[256] = {
    0, KEY_ESC, '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-', '=', KEY_BACKSPACE,
    KEY_TAB, 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', '[', ']', KEY_ENTER,
    KEY_CTRL, 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';', '\'', '`',
    KEY_SHIFT, '\\', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', KEY_SHIFT,
    '*', KEY_ALT, KEY_SPACE, KEY_CAPS, KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, KEY_F7, KEY_F8, KEY_F9, KEY_F10,
    KEY_NUML, KEY_SCROLL, '7', '8', '9', '-', '4', '5', '6', '+', '1', '2', '3', '0', '.', 0, 0, 0, KEY_F11, KEY_F12}; // +0x80 = 释放状态
uint8_t keyboard_code1[256] = {                                                                                        // 按下Shift
    0, KEY_ESC, '!', '@', '#', '$', '%', '^', '&', '*', '(', ')', '_', '+', KEY_BACKSPACE,
    KEY_TAB, 'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P', '{', '}', KEY_ENTER,
    KEY_CTRL, 'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', ':', '\"', '~',
    KEY_SHIFT, '|', 'Z', 'X', 'C', 'V', 'B', 'N', 'M', '<', '>', '?', KEY_SHIFT,
    '*', KEY_ALT, KEY_SPACE, KEY_CAPS, KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, KEY_F7, KEY_F8, KEY_F9, KEY_F10,
    KEY_NUML, KEY_SCROLL, '7', '8', '9', '-', '4', '5', '6', '+', '1', '2', '3', '0', '.', 0, 0, 0, KEY_F11, KEY_F12};

struct keyboard_buf kb_fifo;

hardware_intr_controller kbd_controller = {
    .install = ioapic_add,
    .enable = ioapic_enable,
};

void kbd_handler(uint8_t irq, uint64_t param, struct pt_regs *regs)
{
    uint8_t x = io_in8(0x60);
    parse_scan_code(x);
}

void kbd_init()
{
    kb_fifo.p_head = kb_fifo.buf;
    kb_fifo.p_tail = kb_fifo.buf;
    kb_fifo.count = 0;

    irq_register(PS2_KBD_INTERRUPT_VECTOR, kbd_handler, 0, &kbd_controller, "PS2");
}

void parse_scan_code(uint8_t x)
{
    if (x == 0x2a || x == 0x36)
    {
        kb_fifo.shift = 1;
    }
    else if (x == 0x1d)
    {
        kb_fifo.ctrl = 1;
    }
    else if (x == 0x3a)
    {
        kb_fifo.caps = kb_fifo.caps ^ 1;
    }
    else if (x == 0xaa || x == 0xb6)
    {
        kb_fifo.shift = 0;
    }
    else if (x == 0x9d)
    {
        kb_fifo.ctrl = 0;
    }

    if (kb_fifo.p_head == kb_fifo.buf + KB_BUF_SIZE)
    {
        kb_fifo.p_head = kb_fifo.buf;
    }

    *kb_fifo.p_head = x;
    kb_fifo.count++;
    kb_fifo.p_head++;
}

uint8_t get_keyboard_input()
{
    if (kb_fifo.p_tail != kb_fifo.p_head)
    {
        // 有没有读的
        uint8_t temp = keyboard_code[*kb_fifo.p_tail];
        if (kb_fifo.shift == 1 || kb_fifo.caps == 1)
        {
            temp = keyboard_code1[*kb_fifo.p_tail];
        }

        if (kb_fifo.p_tail == kb_fifo.buf + KB_BUF_SIZE)
        {
            kb_fifo.p_tail = kb_fifo.buf;
        }
        kb_fifo.p_tail++;
        return temp;
    }

    return 0;
}
