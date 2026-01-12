.global _start
_start:
    mov x1, sp
    msr spsel, #1
    mov sp, x1

    b kmain

.global _ap_start
_ap_start:
    mov x1, sp
    msr spsel, #1
    mov sp, x1

    b ap_kmain
