#ifndef __LIB_H__
#define __LIB_H__

#define LIMINE_API_REVISION 3

#include <limine.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <syscall/errno.h>
#include <irq/ptrace.h>

#define container_of(ptr, type, member)                                     \
    ({                                                                      \
        decltype(((type *)0)->member) *p = (ptr);                           \
        (type *)((unsigned long)p - (unsigned long)&(((type *)0)->member)); \
    })

struct List
{
    struct List *prev;
    struct List *next;
};

static inline void list_init(struct List *list)
{
    list->prev = list;
    list->next = list;
}

static inline void list_add_to_behind(struct List *entry, struct List *ne_w) ////add to entry behind
{
    ne_w->next = entry->next;
    ne_w->prev = entry;
    ne_w->next->prev = ne_w;
    entry->next = ne_w;
}

static inline void list_add_to_before(struct List *entry, struct List *ne_w) ////add to entry behind
{
    ne_w->next = entry;
    entry->prev->next = ne_w;
    ne_w->prev = entry->prev;
    entry->prev = ne_w;
}

static inline void list_del(struct List *entry)
{
    entry->next->prev = entry->prev;
    entry->prev->next = entry->next;
}

static inline long list_is_empty(struct List *entry)
{
    if (entry == entry->next && entry->prev == entry)
        return 1;
    else
        return 0;
}

static inline struct List *list_prev(struct List *entry)
{
    if (entry->prev != (struct List *)NULL)
        return entry->prev;
    else
        return (struct List *)NULL;
}

static inline struct List *list_next(struct List *entry)
{
    if (entry->next != (struct List *)NULL)
        return entry->next;
    else
        return (struct List *)NULL;
}

#define ABS(x) ((x) > 0 ? (x) : -(x)) // 绝对值
// 最大最小值
#define max(x, y) ((x > y) ? (x) : (y))
#define min(x, y) ((x < y) ? (x) : (y))

// 四舍五入成整数
static inline uint64_t round(double x)
{
    return (uint64_t)(x + 0.5);
}

#define sti() __asm__ __volatile__("sti	\n\t" ::: "memory")
#define cli() __asm__ __volatile__("cli	\n\t" ::: "memory")
#define nop() __asm__ __volatile__("nop	\n\t")
#define io_mfence() __asm__ __volatile__("mfence	\n\t" ::: "memory")

#define hlt() __asm__ __volatile__("hlt	\n\t")
#define pause() __asm__ __volatile__("pause	\n\t")

static inline void *memcpy(void *To, void *From, long Num)
{
    int d0, d1, d2;
    __asm__ __volatile__("cld	\n\t"
                         "rep	\n\t"
                         "movsq	\n\t"
                         "testb	$4,%b4	\n\t"
                         "je	1f	\n\t"
                         "movsl	\n\t"
                         "1:\ttestb	$2,%b4	\n\t"
                         "je	2f	\n\t"
                         "movsw	\n\t"
                         "2:\ttestb	$1,%b4	\n\t"
                         "je	3f	\n\t"
                         "movsb	\n\t"
                         "3:	\n\t"
                         : "=&c"(d0), "=&D"(d1), "=&S"(d2)
                         : "0"(Num / 8), "q"(Num), "1"(To), "2"(From)
                         : "memory");
    return To;
}

/*
        FirstPart = SecondPart		=>	 0
        FirstPart > SecondPart		=>	 1
        FirstPart < SecondPart		=>	-1
*/

static inline int memcmp(void *FirstPart, void *SecondPart, long Count)
{
    int __res;

    __asm__ __volatile__("cld	\n\t" // clean direct
                         "repe	\n\t" // repeat if equal
                         "cmpsb	\n\t"
                         "je	1f	\n\t"
                         "movl	$1,	%%eax	\n\t"
                         "jl	1f	\n\t"
                         "negl	%%eax	\n\t"
                         "1:	\n\t"
                         : "=a"(__res)
                         : "0"(0), "D"(FirstPart), "S"(SecondPart), "c"(Count)
                         :);
    return __res;
}

/*
        set memory at Address with C ,number is Count
*/

static inline void *memset(void *Address, unsigned char C, long Count)
{
    int d0, d1;
    unsigned long tmp = C * 0x0101010101010101UL;
    __asm__ __volatile__("cld	\n\t"
                         "rep	\n\t"
                         "stosq	\n\t"
                         "testb	$4, %b3	\n\t"
                         "je	1f	\n\t"
                         "stosl	\n\t"
                         "1:\ttestb	$2, %b3	\n\t"
                         "je	2f\n\t"
                         "stosw	\n\t"
                         "2:\ttestb	$1, %b3	\n\t"
                         "je	3f	\n\t"
                         "stosb	\n\t"
                         "3:	\n\t"
                         : "=&c"(d0), "=&D"(d1)
                         : "a"(tmp), "q"(Count), "0"(Count / 8), "1"(Address)
                         : "memory");
    return Address;
}

/*
        string copy
*/

static inline char *strcpy(char *Dest, char *Src)
{
    __asm__ __volatile__("cld	\n\t"
                         "1:	\n\t"
                         "lodsb	\n\t"
                         "stosb	\n\t"
                         "testb	%%al,	%%al	\n\t"
                         "jne	1b	\n\t"
                         :
                         : "S"(Src), "D"(Dest)
                         : "ax", "memory");
    return Dest;
}

/*
        string copy number bytes
*/

static inline char *strncpy(char *Dest, char *Src, long Count)
{
    __asm__ __volatile__("cld	\n\t"
                         "1:	\n\t"
                         "decq	%2	\n\t"
                         "js	2f	\n\t"
                         "lodsb	\n\t"
                         "stosb	\n\t"
                         "testb	%%al,	%%al	\n\t"
                         "jne	1b	\n\t"
                         "rep	\n\t"
                         "stosb	\n\t"
                         "2:	\n\t"
                         :
                         : "S"(Src), "D"(Dest), "c"(Count)
                         : "ax", "memory");
    return Dest;
}

/*
        string cat Dest + Src
*/

static inline char *strcat(char *Dest, char *Src)
{
    __asm__ __volatile__("cld	\n\t"
                         "repne	\n\t"
                         "scasb	\n\t"
                         "decq	%1	\n\t"
                         "1:	\n\t"
                         "lodsb	\n\t"
                         "stosb	\n\r"
                         "testb	%%al,	%%al	\n\t"
                         "jne	1b	\n\t"
                         :
                         : "S"(Src), "D"(Dest), "a"(0), "c"(0xffffffff)
                         : "memory");
    return Dest;
}

/*
        string compare FirstPart and SecondPart
        FirstPart = SecondPart =>  0
        FirstPart > SecondPart =>  1
        FirstPart < SecondPart => -1
*/

static inline int strcmp(char *FirstPart, char *SecondPart)
{
    int __res;
    __asm__ __volatile__("cld	\n\t"
                         "1:	\n\t"
                         "lodsb	\n\t"
                         "scasb	\n\t"
                         "jne	2f	\n\t"
                         "testb	%%al,	%%al	\n\t"
                         "jne	1b	\n\t"
                         "xorl	%%eax,	%%eax	\n\t"
                         "jmp	3f	\n\t"
                         "2:	\n\t"
                         "movl	$1,	%%eax	\n\t"
                         "jl	3f	\n\t"
                         "negl	%%eax	\n\t"
                         "3:	\n\t"
                         : "=a"(__res)
                         : "D"(FirstPart), "S"(SecondPart)
                         :);
    return __res;
}

/*
        string compare FirstPart and SecondPart with Count Bytes
        FirstPart = SecondPart =>  0
        FirstPart > SecondPart =>  1
        FirstPart < SecondPart => -1
*/

static inline int strncmp(char *FirstPart, char *SecondPart, long Count)
{
    int __res;
    __asm__ __volatile__("cld	\n\t"
                         "1:	\n\t"
                         "decq	%3	\n\t"
                         "js	2f	\n\t"
                         "lodsb	\n\t"
                         "scasb	\n\t"
                         "jne	3f	\n\t"
                         "testb	%%al,	%%al	\n\t"
                         "jne	1b	\n\t"
                         "2:	\n\t"
                         "xorl	%%eax,	%%eax	\n\t"
                         "jmp	4f	\n\t"
                         "3:	\n\t"
                         "movl	$1,	%%eax	\n\t"
                         "jl	4f	\n\t"
                         "negl	%%eax	\n\t"
                         "4:	\n\t"
                         : "=a"(__res)
                         : "D"(FirstPart), "S"(SecondPart), "c"(Count)
                         :);
    return __res;
}

static inline int strlen(char *String)
{
    int __res;
    __asm__ __volatile__("cld	\n\t"
                         "repne	\n\t"
                         "scasb	\n\t"
                         "notl	%0	\n\t"
                         "decl	%0	\n\t"
                         : "=c"(__res)
                         : "D"(String), "a"(0), "0"(0xffffffff)
                         :);
    return __res;
}

static inline unsigned long bit_set(unsigned long *addr, unsigned long nr)
{
    return *addr | (1UL << nr);
}

static inline unsigned long bit_get(unsigned long *addr, unsigned long nr)
{
    return *addr & (1UL << nr);
}

static inline unsigned long bit_clean(unsigned long *addr, unsigned long nr)
{
    return *addr & (~(1UL << nr));
}

static inline unsigned char io_in8(unsigned short port)
{
    unsigned char ret = 0;
    __asm__ __volatile__("inb	%%dx,	%0	\n\t"
                         "mfence			\n\t"
                         : "=a"(ret)
                         : "d"(port)
                         : "memory");
    return ret;
}

static inline unsigned int io_in32(unsigned short port)
{
    unsigned int ret = 0;
    __asm__ __volatile__("inl	%%dx,	%0	\n\t"
                         "mfence			\n\t"
                         : "=a"(ret)
                         : "d"(port)
                         : "memory");
    return ret;
}

static inline void io_out8(unsigned short port, unsigned char value)
{
    __asm__ __volatile__("outb	%0,	%%dx	\n\t"
                         "mfence			\n\t"
                         :
                         : "a"(value), "d"(port)
                         : "memory");
}

static inline void io_out32(unsigned short port, unsigned int value)
{
    __asm__ __volatile__("outl	%0,	%%dx	\n\t"
                         "mfence			\n\t"
                         :
                         : "a"(value), "d"(port)
                         : "memory");
}

#define port_insw(port, buffer, nr) \
    __asm__ __volatile__("cld;rep;insw;mfence;" ::"d"(port), "D"(buffer), "c"(nr) : "memory")

#define port_outsw(port, buffer, nr) \
    __asm__ __volatile__("cld;rep;outsw;mfence;" ::"d"(port), "S"(buffer), "c"(nr) : "memory")

static inline unsigned long rdmsr(unsigned long address)
{
    unsigned int tmp0 = 0;
    unsigned int tmp1 = 0;
    __asm__ __volatile__("rdmsr	\n\t" : "=d"(tmp0), "=a"(tmp1) : "c"(address) : "memory");
    return (unsigned long)tmp0 << 32 | tmp1;
}

static inline void wrmsr(unsigned long address, unsigned long value)
{
    __asm__ __volatile__("wrmsr	\n\t" ::"d"(value >> 32), "a"(value & 0xffffffff), "c"(address) : "memory");
}

static inline unsigned long get_rsp()
{
    unsigned long tmp = 0;
    __asm__ __volatile__("movq	%%rsp, %0	\n\t" : "=r"(tmp)::"memory");
    return tmp;
}

static inline unsigned long get_rflags()
{
    unsigned long tmp = 0;
    __asm__ __volatile__("pushfq	\n\t"
                         "movq	(%%rsp), %0	\n\t"
                         "popfq	\n\t"
                         : "=r"(tmp)
                         :
                         : "memory");
    return tmp;
}

static inline long verify_area(unsigned char *addr, unsigned long size)
{
    if (((unsigned long)addr + size) <= (unsigned long)0x00007fffffffffff)
        return 1;
    else
        return 0;
}

typedef struct
{
    volatile int8_t lock; // 1:unlocked 0:locked
} spinlock_t;

static inline void spin_lock(spinlock_t *lock)
{
    __asm__ __volatile__("1:    \n\t"
                         "lock decb %0   \n\t" // 尝试-1
                         "jns 3f    \n\t"      // 加锁成功，跳转到步骤3
                         "2:    \n\t"          // 加锁失败，稍后再试
                         "pause \n\t"
                         "cmpb $0, %0   \n\t"
                         "jle   2b  \n\t" // 若锁被占用，则继续重试
                         "jmp 1b    \n\t" // 尝试加锁
                         "3:"
                         : "=m"(lock->lock)::"memory");
}

static inline void spin_unlock(spinlock_t *lock)
{
    __asm__ __volatile__("movb $1, %0   \n\t"
                         : "=m"(lock->lock)::"memory");
}

static inline void spin_init(spinlock_t *lock)
{
    lock->lock = 1;
}

#endif
