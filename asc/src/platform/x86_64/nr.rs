pub const SYS_PUTSTRING: usize = 1;
pub const SYS_OPEN: usize = 2;
pub const SYS_CLOSE: usize = 3;
pub const SYS_READ: usize = 4;
pub const SYS_WRITE: usize = 5;
pub const SYS_LSEEK: usize = 6;
pub const SYS_FORK: usize = 7;
pub const SYS_VFORK: usize = 8;
pub const SYS_BRK: usize = 9;
pub const SYS_SBRK: usize = 10;

pub const SYS_REBOOT: usize = 11;
pub const SYS_CHDIR: usize = 12;
pub const SYS_GET_DENTS: usize = 13;
pub const SYS_EXECVE: usize = 14;
pub const SYS_WAIT4: usize = 15;
pub const SYS_EXIT: usize = 16;
pub const SYS_MKDIR: usize = 17;
pub const SYS_NANOSLEEP: usize = 18;
pub const SYS_CLOCK: usize = 19;
pub const SYS_PIPE: usize = 20;

pub const SYS_MSTAT: usize = 21;
pub const SYS_UNLINK_AT: usize = 22;
pub const SYS_KILL: usize = 23;
pub const SYS_SIGACTION: usize = 24;
pub const SYS_RT_SIGRETURN: usize = 25;
pub const SYS_GETPID: usize = 26;
pub const SYS_DUP: usize = 28;
pub const SYS_DUP2: usize = 29;
pub const SYS_SOCKET: usize = 30;

pub const SYS_SETSOCKOPT: usize = 31;
pub const SYS_GETSOCKOPT: usize = 32;
pub const SYS_CONNECT: usize = 33;
pub const SYS_BIND: usize = 34;
pub const SYS_SENDTO: usize = 35;
pub const SYS_RECVFROM: usize = 36;
pub const SYS_RECVMSG: usize = 37;
pub const SYS_LISTEN: usize = 38;
pub const SYS_SHUTDOWN: usize = 39;
pub const SYS_ACCEPT: usize = 40;

pub const SYS_GETSOCKNAME: usize = 41;
pub const SYS_GETPEERNAME: usize = 42;

pub const SYS_GETTIMEOFDAY: usize = 43;

pub const SYS_IOPL: usize = 44;
pub const SYS_VIRTTOPHYS: usize = 45;
