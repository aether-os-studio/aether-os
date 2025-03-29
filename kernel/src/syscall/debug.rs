use core::ffi::CStr;

use alloc::{
    string::{String, ToString},
    vec::Vec,
};
use asc::nr::*;
use error::SystemError;

struct ByteStr<'a>(&'a [u8]);

impl<'a> ::core::fmt::Debug for ByteStr<'a> {
    fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
        write!(f, "\"")?;
        for i in self.0 {
            for ch in core::ascii::escape_default(*i) {
                write!(f, "{}", ch as char)?;
            }
        }
        write!(f, "\"")?;
        Ok(())
    }
}

fn debug_path(ptr: usize) -> Result<String, SystemError> {
    let cstr = unsafe { CStr::from_ptr(ptr as *const i8) };
    cstr.to_str()
        .map(|s| s.to_string())
        .or(Err(SystemError::EINVAL))
}

fn debug_buf(ptr: usize, len: usize) -> Result<Vec<u8>, SystemError> {
    let user = unsafe { core::slice::from_raw_parts(ptr as *const u8, len) };
    let mut buf = vec![0_u8; 4096];
    buf.copy_from_slice(user);
    Ok(buf)
}

//TODO: calling format_call with arguments from another process space will not work
pub fn format_call(a: usize, b: usize, c: usize, d: usize, e: usize, f: usize) -> String {
    match a {
        OPEN => format!(
            "open({:?}, {:#X})",
            debug_path(b).as_ref().map(|p| ByteStr(p.as_bytes())),
            d
        ),
        RMDIR => format!(
            "rmdir({:?})",
            debug_path(b).as_ref().map(|p| ByteStr(p.as_bytes())),
        ),
        UNLINK => format!(
            "unlink({:?})",
            debug_path(b).as_ref().map(|p| ByteStr(p.as_bytes())),
        ),
        CLOSE => format!("close({})", b),
        DUP => format!(
            "dup({}, {:?})",
            b,
            debug_buf(c, d).as_ref().map(|b| ByteStr(&*b)),
        ),
        DUP2 => format!(
            "dup2({}, {}, {:?})",
            b,
            c,
            debug_buf(d, e).as_ref().map(|b| ByteStr(&*b)),
        ),
        READ => format!("read({}, {:#X}, {})", b, c, d),
        WRITE => format!("write({}, {:#X}, {})", b, c, d),
        FCHMOD => format!("fchmod({}, {:#o})", b, c),
        FCHOWN => format!("fchown({}, {}, {})", b, c, d),
        FSYNC => format!("fsync({})", b),
        FTRUNCATE => format!("ftruncate({}, {})", b, c),
        EXIT => format!("exit({})", b),
        GETEGID => format!("getegid()"),
        GETEUID => format!("geteuid()"),
        GETGID => format!("getgid()"),
        GETPGID => format!("getpgid()"),
        GETPID => format!("getpid()"),
        GETPPID => format!("getppid()"),
        GETUID => format!("getuid()"),
        IOPL => format!("iopl({})", b),
        KILL => format!("kill({}, {})", b, c),
        SETREGID => format!("setregid({}, {})", b, c),
        SETREUID => format!("setreuid({}, {})", b, c),
        _ => format!(
            "UNKNOWN{} {:#X}({:#X}, {:#X}, {:#X}, {:#X}, {:#X})",
            a, a, b, c, d, e, f
        ),
    }
}
