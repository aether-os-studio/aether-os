/// 表示时间的结构体，符合POSIX标准。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(C)]
pub struct PosixTimeSpec {
    pub tv_sec: i64,
    pub tv_nsec: i64,
}

impl PosixTimeSpec {
    #[allow(dead_code)]
    pub fn new(sec: i64, nsec: i64) -> PosixTimeSpec {
        return PosixTimeSpec {
            tv_sec: sec,
            tv_nsec: nsec,
        };
    }

    pub fn total_nanos(&self) -> i64 {
        self.tv_sec * 1000000000 + self.tv_nsec
    }
}
