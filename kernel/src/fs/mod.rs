use alloc::{string::String, sync::Arc, vec::Vec};
use fat::Fat32Volume;
use spin::{Lazy, Mutex, RwLock};
use vfs::{IndexNode, IndexNodeRef, fake::FakeFS, partition::PartitionIndexNode};

use crate::{
    arch::{arch_disable_intr, arch_enable_intr, arch_yield},
    drivers::{
        base::input::BYTES, block::partition::PARTITION_DEVICES, display::FRAMEBUFFER_REQUEST,
        term::TERMINAL,
    },
    errno::Errno,
    fs::syscall::POLLIN,
    print,
    proc::sched::get_current_context,
    serial_print,
    syscall::Result,
};

pub mod fat;
pub mod fd;
pub mod path_walk;
pub mod syscall;
pub mod vfs;

pub const DT_UNKNOWN: u8 = 0;
pub const DT_FIFO: u8 = 1;
pub const DT_CHR: u8 = 2;
pub const DT_DIR: u8 = 4;
pub const DT_BLK: u8 = 6;
pub const DT_REG: u8 = 8;
pub const DT_LNK: u8 = 10;
pub const DT_SOCK: u8 = 12;
pub const DT_WHT: u8 = 14;

pub const F_DUPFD: usize = 0;
pub const F_GETFD: usize = 1;
pub const F_SETFD: usize = 2;
pub const F_GETFL: usize = 3;
pub const F_SETFL: usize = 4;
pub const F_SETOWN: usize = 8;
pub const F_GETOWN: usize = 9;
pub const F_SETSIG: usize = 10;
pub const F_GETSIG: usize = 11;

pub const F_DUPFD_CLOEXEC: usize = 1030;

#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct PosixStat {
    pub st_dev: usize,
    pub st_ino: usize,
    pub st_nlink: usize,
    pub st_mode: u32,
    pub st_uid: u32,
    pub st_gid: u32,
    pub __pad0: u32,
    pub st_rdev: usize,
    pub st_size: isize,
    pub st_blksize: isize,
    /// number of 512B blocks allocated
    pub st_blocks: isize,
    pub st_atime: usize,
    pub st_atime_nsec: usize,
    pub st_mtime: usize,
    pub st_mtime_nsec: usize,
    pub st_ctime: usize,
    pub st_ctime_nsec: usize,
    pub __unused: [isize; 3],
}

bitflags::bitflags! {

    ///  The constants AT_REMOVEDIR and AT_EACCESS have the same value.  AT_EACCESS is
    ///  meaningful only to faccessat, while AT_REMOVEDIR is meaningful only to
    ///  unlinkat.  The two functions do completely different things and therefore,
    ///  the flags can be allowed to overlap.  For example, passing AT_REMOVEDIR to
    ///  faccessat would be undefined behavior and thus treating it equivalent to
    ///  AT_EACCESS is valid undefined behavior.
    #[allow(clippy::bad_bit_mask)]
    pub struct AtFlags: i32 {
        /// 特殊值，用于指示openat应使用当前工作目录。
        const AT_FDCWD = -100;
        /// 不要跟随符号链接。
        const AT_SYMLINK_NOFOLLOW = 0x100;
        /// AtEAccess: 使用有效ID进行访问测试，而不是实际ID。
        const AT_EACCESS = 0x200;
        /// AtRemoveDir: 删除目录而不是取消链接文件。
        const AT_REMOVEDIR = 0x200;

        /// 跟随符号链接。
        /// AT_SYMLINK_FOLLOW: 0x400
        const AT_SYMLINK_FOLLOW = 0x400;
        /// 禁止终端自动挂载遍历。
        /// AT_NO_AUTOMOUNT: 0x800
        const AT_NO_AUTOMOUNT = 0x800;
        /// 允许空的相对路径名。
        /// AT_EMPTY_PATH: 0x1000
        const AT_EMPTY_PATH = 0x1000;
        /// statx()所需的同步类型。
        /// AT_STATX_SYNC_TYPE: 0x6000
        const AT_STATX_SYNC_TYPE = 0x6000;
        /// 执行与stat()相同的操作。
        /// AT_STATX_SYNC_AS_STAT: 0x0000
        const AT_STATX_SYNC_AS_STAT = 0x0000;
        /// 强制将属性与服务器同步。
        /// AT_STATX_FORCE_SYNC: 0x2000
        const AT_STATX_FORCE_SYNC = 0x2000;
        /// 不要将属性与服务器同步。
        /// AT_STATX_DONT_SYNC: 0x4000
        const AT_STATX_DONT_SYNC = 0x4000;
        /// 应用于整个子树。
        /// AT_RECURSIVE: 0x8000
        const AT_RECURSIVE = 0x8000;

        const AT_GETATTR_NOSEC = i32::MIN;
    }
}

pub struct StdioIndexNode {
    path: String,
}

impl StdioIndexNode {
    pub fn new() -> IndexNodeRef {
        Arc::new(RwLock::new(StdioIndexNode {
            path: String::new(),
        }))
    }
}

pub const TCGETS: usize = 0x5401;
pub const TCSETS: usize = 0x5402;
pub const TCSETSW: usize = 0x5403;
pub const TCSETSF: usize = 0x5404;
pub const TCGETA: usize = 0x5405;
pub const TCSETA: usize = 0x5406;
pub const TCSETAW: usize = 0x5407;
pub const TCSETAF: usize = 0x5408;
pub const TCSBRK: usize = 0x5409;
pub const TCXONC: usize = 0x540A;
pub const TCFLSH: usize = 0x540B;
pub const TIOCEXCL: usize = 0x540C;
pub const TIOCNXCL: usize = 0x540D;
pub const TIOCSCTTY: usize = 0x540E;
pub const TIOCGPGRP: usize = 0x540F;
pub const TIOCSPGRP: usize = 0x5410;
pub const TIOCOUTQ: usize = 0x5411;
pub const TIOCSTI: usize = 0x5412;
pub const TIOCGWINSZ: usize = 0x5413;
pub const TIOCSWINSZ: usize = 0x5414;
pub const TIOCMGET: usize = 0x5415;
pub const TIOCMBIS: usize = 0x5416;
pub const TIOCMBIC: usize = 0x5417;
pub const TIOCMSET: usize = 0x5418;
pub const TIOCGSOFTCAR: usize = 0x5419;
pub const TIOCSSOFTCAR: usize = 0x541A;
pub const FIONREAD: usize = 0x541B;
pub const TIOCINQ: usize = FIONREAD;
pub const TIOCLINUX: usize = 0x541C;
pub const TIOCCONS: usize = 0x541D;
pub const TIOCGSERIAL: usize = 0x541E;
pub const TIOCSSERIAL: usize = 0x541F;
pub const TIOCPKT: usize = 0x5420;
pub const FIONBIO: usize = 0x5421;
pub const TIOCNOTTY: usize = 0x5422;
pub const TIOCSETD: usize = 0x5423;
pub const TIOCGETD: usize = 0x5424;
pub const TCSBRKP: usize = 0x5425;
pub const TIOCSBRK: usize = 0x5427;
pub const TIOCCBRK: usize = 0x5428;
pub const TIOCGSID: usize = 0x5429;
pub const TIOCGRS485: usize = 0x542E;
pub const TIOCSRS485: usize = 0x542F;
pub const TIOCGPTN: usize = 0x80045430;
pub const TIOCSPTLCK: usize = 0x40045431;
pub const TIOCGDEV: usize = 0x80045432;
pub const TCGETX: usize = 0x5432;
pub const TCSETX: usize = 0x5433;
pub const TCSETXF: usize = 0x5434;
pub const TCSETXW: usize = 0x5435;
pub const TIOCSIG: usize = 0x40045436;
pub const TIOCVHANGUP: usize = 0x5437;
pub const TIOCGPKT: usize = 0x80045438;
pub const TIOCGPTLCK: usize = 0x80045439;
pub const TIOCGEXCL: usize = 0x80045440;
pub const TIOCGPTPEER: usize = 0x5441;
pub const TIOCGISO7816: usize = 0x80285442;
pub const TIOCSISO7816: usize = 0xc0285443;

pub const FIONCLEX: usize = 0x5450;
pub const FIOCLEX: usize = 0x5451;
pub const FIOASYNC: usize = 0x5452;
pub const TIOCSERCONFIG: usize = 0x5453;
pub const TIOCSERGWILD: usize = 0x5454;
pub const TIOCSERSWILD: usize = 0x5455;
pub const TIOCGLCKTRMIOS: usize = 0x5456;
pub const TIOCSLCKTRMIOS: usize = 0x5457;
pub const TIOCSERGSTRUCT: usize = 0x5458;
pub const TIOCSERGETLSR: usize = 0x5459;
pub const TIOCSERGETMULTI: usize = 0x545A;
pub const TIOCSERSETMULTI: usize = 0x545B;

pub const TIOCMIWAIT: usize = 0x545C;
pub const TIOCGICOUNT: usize = 0x545D;
pub const FIOQSIZE: usize = 0x5460;

pub const TIOCM_LE: usize = 0x001;
pub const TIOCM_DTR: usize = 0x002;
pub const TIOCM_RTS: usize = 0x004;
pub const TIOCM_ST: usize = 0x008;
pub const TIOCM_SR: usize = 0x010;
pub const TIOCM_CTS: usize = 0x020;
pub const TIOCM_CAR: usize = 0x040;
pub const TIOCM_RNG: usize = 0x080;
pub const TIOCM_DSR: usize = 0x100;
pub const TIOCM_CD: usize = TIOCM_CAR;
pub const TIOCM_RI: usize = TIOCM_RNG;
pub const TIOCM_OUT1: usize = 0x2000;
pub const TIOCM_OUT2: usize = 0x4000;
pub const TIOCM_LOOP: usize = 0x8000;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct WinSize {
    pub ws_row: u16,
    pub ws_col: u16,
    pub ws_xpixel: u16,
    pub ws_ypixel: u16,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Termios {
    pub c_iflag: u32, // 输入模式标志
    pub c_oflag: u32, // 输出模式标志
    pub c_cflag: u32, // 控制模式标志
    pub c_lflag: u32, // 本地模式标志
    pub c_line: u8,
    pub c_cc: [u8; 32], // 控制字符
}

pub const VINTR: usize = 0;
pub const VQUIT: usize = 1;
pub const VERASE: usize = 2;
pub const VKILL: usize = 3;
pub const VEOF: usize = 4;
pub const VTIME: usize = 5;
pub const VMIN: usize = 6;
pub const VSWTC: usize = 7;
pub const VSTART: usize = 8;
pub const VSTOP: usize = 9;
pub const VSUSP: usize = 10;
pub const VEOL: usize = 11;
pub const VREPRINT: usize = 12;
pub const VDISCARD: usize = 13;
pub const VWERASE: usize = 14;
pub const VLNEXT: usize = 15;
pub const VEOL2: usize = 16;

pub const IGNBRK: usize = 0000001;
pub const BRKINT: usize = 0000002;
pub const IGNPAR: usize = 0000004;
pub const PARMRK: usize = 0000010;
pub const INPCK: usize = 0000020;
pub const ISTRIP: usize = 0000040;
pub const INLCR: usize = 0000100;
pub const IGNCR: usize = 0000200;
pub const ICRNL: usize = 0000400;
pub const IUCLC: usize = 0001000;
pub const IXON: usize = 0002000;
pub const IXANY: usize = 0004000;
pub const IXOFF: usize = 0010000;
pub const IMAXBEL: usize = 0020000;
pub const IUTF8: usize = 0040000;

pub const OPOST: usize = 0000001;
pub const OLCUC: usize = 0000002;
pub const ONLCR: usize = 0000004;
pub const OCRNL: usize = 0000010;
pub const ONOCR: usize = 0000020;
pub const ONLRET: usize = 0000040;
pub const OFILL: usize = 0000100;
pub const OFDEL: usize = 0000200;
pub const NLDLY: usize = 0000400;
pub const NL0: usize = 0000000;
pub const NL1: usize = 0000400;
pub const CRDL: usize = 0003000;
pub const CR0: usize = 0000000;
pub const CR1: usize = 0001000;
pub const CR2: usize = 0002000;
pub const CR3: usize = 0003000;
pub const TABDLY: usize = 0014000;
pub const TAB0: usize = 0000000;
pub const TAB1: usize = 0004000;
pub const TAB2: usize = 0010000;
pub const TAB3: usize = 0014000;
pub const BSDLY: usize = 0020000;
pub const BS0: usize = 0000000;
pub const BS1: usize = 0020000;
pub const FFDLY: usize = 0100000;
pub const FF0: usize = 0000000;
pub const FF1: usize = 0100000;

pub const VTDLY: usize = 0040000;
pub const VT0: usize = 0000000;
pub const VT1: usize = 0040000;

pub const B0: usize = 0000000;
pub const B50: usize = 0000001;
pub const B75: usize = 0000002;
pub const B110: usize = 0000003;
pub const B134: usize = 0000004;
pub const B150: usize = 0000005;
pub const B200: usize = 0000006;
pub const B300: usize = 0000007;
pub const B600: usize = 0000010;
pub const B1200: usize = 0000011;
pub const B1800: usize = 0000012;
pub const B2400: usize = 0000013;
pub const B4800: usize = 0000014;
pub const B9600: usize = 0000015;
pub const B19200: usize = 0000016;
pub const B38400: usize = 0000017;

pub const B57600: usize = 0010001;
pub const B115200: usize = 0010002;
pub const B230400: usize = 0010003;
pub const B460800: usize = 0010004;
pub const B500000: usize = 0010005;
pub const B576000: usize = 0010006;
pub const B921600: usize = 0010007;
pub const B1000000: usize = 0010010;
pub const B1152000: usize = 0010011;
pub const B1500000: usize = 0010012;
pub const B2000000: usize = 0010013;
pub const B2500000: usize = 0010014;
pub const B3000000: usize = 0010015;
pub const B3500000: usize = 0010016;
pub const B4000000: usize = 0010017;

pub const CSIZE: usize = 0000060;
pub const CS5: usize = 0000000;
pub const CS6: usize = 0000020;
pub const CS7: usize = 0000040;
pub const CS8: usize = 0000060;
pub const CSTOPB: usize = 0000100;
pub const CREAD: usize = 0000200;
pub const PARENB: usize = 0000400;
pub const PARODD: usize = 0001000;
pub const HUPCL: usize = 0002000;
pub const CLOCAL: usize = 0004000;

pub const ISIG: usize = 0000001;
pub const ICANON: usize = 0000002;
pub const ECHO: usize = 0000010;
pub const ECHOE: usize = 0000020;
pub const ECHOK: usize = 0000040;
pub const ECHONL: usize = 0000100;
pub const NOFLSH: usize = 0000200;
pub const TOSTOP: usize = 0000400;
pub const IEXTEN: usize = 0100000;

pub const TCOOFF: usize = 0;
pub const TCOON: usize = 1;
pub const TCIOFF: usize = 2;
pub const TCION: usize = 3;

pub const TCIFLUSH: usize = 0;
pub const TCOFLUSH: usize = 1;
pub const TCIOFLUSH: usize = 2;

pub const TCSANOW: usize = 0;
pub const TCSADRAIN: usize = 1;
pub const TCSAFLUSH: usize = 2;

pub const EXTA: usize = 0000016;
pub const EXTB: usize = 0000017;
pub const CBAUD: usize = 0010017;
pub const CBAUDEX: usize = 0010000;
pub const CIBAUD: usize = 002003600000;
pub const CMSPAR: usize = 010000000000;
pub const CRTSCTS: usize = 020000000000;

pub const XCASE: usize = 0000004;
pub const ECHOCTL: usize = 0001000;
pub const ECHOPRT: usize = 0002000;
pub const ECHOKE: usize = 0004000;
pub const FLUSHO: usize = 0010000;
pub const PENDIN: usize = 0040000;
pub const EXTPROC: usize = 0200000;

pub const XTABS: usize = 0014000;

impl IndexNode for StdioIndexNode {
    fn when_mounted(&mut self, path: String, father: Option<IndexNodeRef>) {
        self.path.clear();
        self.path.push_str(path.as_str());
    }

    fn when_umounted(&mut self) {}

    fn get_info(&self) -> vfs::IndexNodeInfo {
        let mut index_node_info = vfs::IndexNodeInfo::default();
        index_node_info.path = self.path.clone();
        index_node_info.inode_type = vfs::IndexNodeType::File;
        index_node_info
    }

    fn read_at(&self, _offset: usize, buf: &mut [u8]) -> crate::syscall::Result<usize> {
        let mut bytes_written = 0;
        let ctx = get_current_context();
        let echo = ctx.read().termios.c_lflag & (ECHO as u32) != 0;
        let icanon = ctx.read().termios.c_lflag & (ICANON as u32) != 0;
        let mut line_buffer = Vec::new();

        loop {
            let byte = BYTES.pop();

            if let Some(byte) = byte {
                if icanon {
                    match byte {
                        b'\n' => {
                            line_buffer.push(byte);
                            buf[bytes_written..bytes_written + line_buffer.len()]
                                .copy_from_slice(&line_buffer);
                            bytes_written += line_buffer.len();
                            break;
                        }
                        8 => {
                            if !line_buffer.is_empty() {
                                line_buffer.pop();
                                if echo {
                                    self.write_at(0, &[8])?; // 回显退格效果
                                }
                            }
                            continue;
                        }
                        _ => {
                            line_buffer.push(byte);
                            if echo {
                                self.write_at(0, &[byte])?;
                            }
                        }
                    }
                } else {
                    buf[bytes_written] = byte;
                    bytes_written += 1;
                    if echo {
                        self.write_at(0, &[byte])?;
                    }
                }

                if bytes_written >= buf.len() {
                    break;
                }
            } else {
                arch_enable_intr();
                arch_yield();
                arch_disable_intr();
            }
        }

        Ok(bytes_written)
    }

    fn write_at(&self, _offset: usize, buf: &[u8]) -> crate::syscall::Result<usize> {
        let str = unsafe { str::from_utf8_unchecked(buf) };
        serial_print!("{}", str);
        print!("{}", str);
        Ok(buf.len())
    }

    fn ioctl(&self, cmd: usize, arg: usize) -> Result<usize> {
        match cmd {
            TIOCGWINSZ => {
                let fb = FRAMEBUFFER_REQUEST
                    .get_response()
                    .unwrap()
                    .framebuffers()
                    .next()
                    .unwrap();
                let wsz = arg as *mut WinSize;
                unsafe {
                    (*wsz).ws_col = TERMINAL.lock().columns() as u16;
                    (*wsz).ws_row = TERMINAL.lock().rows() as u16;
                    (*wsz).ws_xpixel = fb.width() as u16;
                    (*wsz).ws_ypixel = fb.height() as u16;
                };
                Ok(0)
            }
            TIOCSWINSZ => return Ok(0),
            TIOCGPGRP => {
                let pid = arg as *mut i32;
                unsafe { (*pid) = get_current_context().read().get_pid() as i32 };
                Ok(0)
            }
            TIOCSPGRP => return Ok(0),
            TCGETS => {
                let termios_ptr = arg as *mut Termios;
                unsafe {
                    *termios_ptr = get_current_context().read().termios;
                }
                Ok(0)
            }
            TCSETS => {
                let termios_ptr = arg as *const Termios;
                unsafe {
                    get_current_context().write().termios = *termios_ptr;
                }
                Ok(0)
            }
            TCSETSW => {
                let termios_ptr = arg as *const Termios;
                unsafe {
                    get_current_context().write().termios = *termios_ptr;
                }
                Ok(0)
            }
            _ => return Err(Errno::ENOSYS),
        }
    }

    fn poll(&self, event: usize) -> Result<usize> {
        let mut res = 0;
        if (event & POLLIN) != 0 {
            res |= POLLIN;
        }
        Ok(res)
    }
}

pub static ROOT: Lazy<Mutex<IndexNodeRef>> = Lazy::new(|| Mutex::new(FakeFS::new()));

pub fn init() {
    info!("Found {} partitions", PARTITION_DEVICES.lock().len());
    for partition in PARTITION_DEVICES.lock().iter() {
        if let Some(root) = Fat32Volume::new(PartitionIndexNode::new(partition.clone())) {
            *ROOT.lock() = root;
            info!("Mount root OK");
            break;
        }
    }
}
