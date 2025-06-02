/// 表示时间的结构体，符合POSIX标准。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(C)]
pub struct PosixTimeSpec {
    pub tv_sec: i64,
    pub tv_nsec: i64,
}
