.section .text

.global restorer

restorer:
    addq $0x8, %rsp
    movq (%rsp), %rdi
    call ssetmask
    addq $0x8, %rsp

    // === 恢复调用现场 ===
    popq %r15
    popq %r14
    popq %r13
    popq %r12
    popq %r11
    popq %r10
    popq %r9
    popq %r8
    popq %rbx
    popq %rcx
    popq %rdx
    popq %rsi
    popq %rdi
    popq %rbp

    popq %rax   //  不允许直接pop到ds
    movq %rax, %ds

    popq %rax
    movq %rax, %es

    popq %rax
    addq $0x10, %rsp // 弹出变量FUNC和errcode

    ret
