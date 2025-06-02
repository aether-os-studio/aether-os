#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum AtType {
    NULL = 0,
    IGNORE,
    EXECFD,
    PHDR,
    PHENT,
    PHNUM,
    PAGESZ,
    BASE,
    FLAGS,
    ENTRY,
    NOTELF,
    UID,
    EUID,
    GID,
    EGID,
    CLKTCK,
    PLATFORM,
    HWCAP,
    FPUCW = 18,
    DCACHEBSIZE,
    ICACHEBSIZE,
    UCACHEBSIZE,
    IGNOREPPC,
    SECURE,
    BASEPLATFORM,
    RANDOM,
    HWCAP2,
    EXECFN = 31,
    SYSINFO,
    SYSINFOEHDR,
}
bitflags::bitflags! {
    pub struct WaitOption: u32{
        const WNOHANG = 0x00000001;
        const WUNTRACED = 0x00000002;
        const WSTOPPED = 0x00000002;
        const WEXITED = 0x00000004;
        const WCONTINUED = 0x00000008;
        const WNOWAIT = 0x01000000;
        const WNOTHREAD = 0x20000000;
        const WALL = 0x40000000;
        const WCLONE = 0x80000000;
    }
}
