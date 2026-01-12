.macro SAVE_CONTEXT
    sub sp, sp, #272
    
    stp x0, x1, [sp, #256]
    stp x2, x3, [sp, #240]
    stp x4, x5, [sp, #224]
    stp x6, x7, [sp, #208]
    stp x8, x9, [sp, #192]
    stp x10, x11, [sp, #176]
    stp x12, x13, [sp, #160]
    stp x14, x15, [sp, #144]
    stp x16, x17, [sp, #128]
    stp x18, x19, [sp, #112]
    stp x20, x21, [sp, #96]
    stp x22, x23, [sp, #80]
    stp x24, x25, [sp, #64]
    stp x26, x27, [sp, #48]
    stp x28, x29, [sp, #32]
    
    mrs x0, ELR_EL1
    mrs x1, SPSR_EL1
    stp x0, x1, [sp, #0]
    mrs x0, SP_EL0
    stp x0, x30, [sp, #16]
.endm

.macro RESTORE_CONTEXT
    ldp x0, x1, [sp, #0]
    msr ELR_EL1, x0
    msr SPSR_EL1, x1
    ldp x0, x30, [sp, #16]
    msr SP_EL0, x0
    
    ldp x28, x29, [sp, #32]
    ldp x26, x27, [sp, #48]
    ldp x24, x25, [sp, #64]
    ldp x22, x23, [sp, #80]
    ldp x20, x21, [sp, #96]
    ldp x18, x19, [sp, #112]
    ldp x16, x17, [sp, #128]
    ldp x14, x15, [sp, #144]
    ldp x12, x13, [sp, #160]
    ldp x10, x11, [sp, #176]
    ldp x8, x9, [sp, #192]
    ldp x6, x7, [sp, #208]
    ldp x4, x5, [sp, #224]
    ldp x2, x3, [sp, #240]
    ldp x0, x1, [sp, #256]
    
    add sp, sp, #272
    eret
.endm

.align 11  // 2KB 对齐 (2^11 = 2048)
.global exception_vectors
exception_vectors:
.align 7
sync_el1_sp0:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_sync_el1_sp0
    RESTORE_CONTEXT
.align 7
irq_el1_sp0:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_irq_el1_sp0
    RESTORE_CONTEXT
.align 7
fiq_el1_sp0:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_fiq_el1_sp0
    RESTORE_CONTEXT
.align 7
serror_el1_sp0:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_serror_el1_sp0
    RESTORE_CONTEXT

// Current EL with SPx
.align 7
sync_el1_spx:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_sync_el1_spx
    RESTORE_CONTEXT
.align 7
irq_el1_spx:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_irq_el1_spx
    RESTORE_CONTEXT
.align 7
fiq_el1_spx:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_fiq_el1_spx
    RESTORE_CONTEXT
.align 7
serror_el1_spx:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_serror_el1_spx
    RESTORE_CONTEXT

// Lower EL (AArch64)
.align 7
sync_el0_64:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_sync_el0_64
    RESTORE_CONTEXT
.align 7
irq_el0_64:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_irq_el0_64
    RESTORE_CONTEXT
.align 7
fiq_el0_64:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_fiq_el0_64
    RESTORE_CONTEXT
.align 7
serror_el0_64:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_serror_el0_64
    RESTORE_CONTEXT

// Lower EL (AArch32)
.align 7
sync_el0_32:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_sync_el0_32
    RESTORE_CONTEXT
.align 7
irq_el0_32:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_irq_el0_32
    RESTORE_CONTEXT
.align 7
fiq_el0_32:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_fiq_el0_32
    RESTORE_CONTEXT
.align 7
serror_el0_32:
    SAVE_CONTEXT
    mov x0, sp
    bl handle_serror_el0_32
    RESTORE_CONTEXT
