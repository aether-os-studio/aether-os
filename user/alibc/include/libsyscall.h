#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <errno.h>

enum
{
    SYS_READ = 1,
    SYS_WRITE,
    SYS_OPEN,
    SYS_CLOSE,
    SYS_IOCTL,
    SYS_SIGACTION,
    SYS_SIGNAL,
    SYS_SETMASK,
    SYS_SENDSIGNAL,
    SYS_EXIT,
    SYS_GETPID,
    SYS_GETPPID,
    SYS_FORK,
    SYS_WAITPID,
    SYS_LOAD_MODULE,
    SYS_IOPL,

    SYS_GET_INFO,

    SYS_BRK,
    SYS_PHYSMAP,
    SYS_VIRTTOPHYS,

    SYS_SCHEME_CREATE,

    SYS_DMA_ALLOC,
    SYS_DMA_FREE,

    SYS_NUM,
};

typedef struct bootstrap_info
{
    // About ACPI
    uint64_t rsdp_phys_address;
    // About framebuffer
    uint64_t framebuffer_address;
    uint64_t framebuffer_width;
    uint64_t framebuffer_height;
    uint8_t red_mask_size;
    uint8_t red_mask_shift;
    uint8_t green_mask_size;
    uint8_t green_mask_shift;
    uint8_t blue_mask_size;
    uint8_t blue_mask_shift;
} bootstrap_info_t;

#define PROT_READ (1UL << 0)
#define PROT_WRITE (1UL << 1)
#define PROT_EXEC (1UL << 2)

uint64_t enter_syscall(uint64_t idx, uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5, uint64_t arg6);

enum SIGNAL
{
    SIGHUP = 1,   // 挂断控制终端或进程
    SIGINT,       // 来自键盘的中断
    SIGQUIT,      // 来自键盘的退出
    SIGILL,       // 非法指令
    SIGTRAP,      // 跟踪断点
    SIGABRT,      // 异常结束
    SIGIOT = 6,   // 异常结束
    SIGUNUSED,    // 没有使用
    SIGFPE,       // 协处理器出错
    SIGKILL = 9,  // 强迫进程终止
    SIGUSR1,      // 用户信号 1，进程可使用
    SIGSEGV,      // 无效内存引用
    SIGUSR2,      // 用户信号 2，进程可使用
    SIGPIPE,      // 管道写出错，无读者
    SIGALRM,      // 实时定时器报警
    SIGTERM = 15, // 进程终止
    SIGSTKFLT,    // 栈出错（协处理器）
    SIGCHLD,      // 子进程停止或被终止
    SIGCONT,      // 恢复进程继续执行
    SIGSTOP,      // 停止进程的执行
    SIGTSTP,      // tty 发出停止进程，可忽略
    SIGTTIN,      // 后台进程请求输入
    SIGTTOU = 22, // 后台进程请求输出
};

typedef unsigned long long sigset_t;

// 信号处理结构
typedef struct sigaction_t
{
    void (*handler)(int); // 信号处理函数
    sigset_t mask;        // 信号屏蔽码
    uint64_t flags;
    void (*restorer)(void); // 恢复函数指针
} sigaction_t;

extern void restorer();

int signal(int sig, uint64_t handler);

int sigaction(int sg, sigaction_t *act, sigaction_t *oldact);

void send_signal(int pid, int sig);

int getpid();
int getppid();

int open(const char *name, int mode, int flags);
int read(int fd, void *buf, int len);
int write(int fd, void *buf, int len);
int close(int fd);
int ioctl(int fd, int cmd, int arg);

void iopl(uint64_t level);

int fork();

int waitpid(int pid, int *status);

uint64_t virttophys(uint64_t virt);

void load_module(const char *name);

void get_bootstrap_info(bootstrap_info_t *info);

uint64_t physmap(uint64_t addr, uint64_t size, uint64_t flags);

#include <stdlib.h>

#define SCHEME_NAME_MAX 128

enum
{
    SCHEME_IOCTL_GETSIZE = 1,
    SCHEME_IOCTL_GETBLKSIZE,
    SCHEME_IOCTL_REGIST_BLKDEV,
};

enum
{
    SCHEME_COMMAND_READ = 1,
    SCHEME_COMMAND_WRITE,
    SCHEME_COMMAND_IOCTL,
};

typedef struct user_scheme_command
{
    uint64_t cmd;
    uint64_t a;
    uint64_t b;
    uint64_t c;
    uint64_t d;
    uint64_t e;
    uint64_t f;
} user_scheme_command_t;

typedef struct user_scheme
{
    user_scheme_command_t command;
} user_scheme_t;

static inline void init_scheme(user_scheme_t *scheme)
{
    memset(scheme, 0, sizeof(user_scheme_t));
    scheme->command.d = malloc(SCHEME_NAME_MAX);
    memset((void *)scheme->command.d, 0, SCHEME_NAME_MAX);
}

void scheme_create(const char *name, user_scheme_t *addr);

uint64_t alloc_dma(uint64_t count);
void free_dma(uint64_t addr, uint64_t count);
