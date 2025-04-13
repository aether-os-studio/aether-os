#pragma once

enum
{
    SYS_READ = 1,
    SYS_WRITE,
    SYS_OPEN,
    SYS_CLOSE,
    SYS_SIGACTION,
    SYS_SIGNAL,
    SYS_SETMASK,
    SYS_SENDSIGNAL,
    SYS_EXIT,
    SYS_GETPID,
    SYS_FORK,
    SYS_IOPL,

    SYS_NUM,
};

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;

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

int read(int fd, void *buf, int len);
int write(int fd, void *buf, int len);

void iopl(uint64_t level);

int fork();
