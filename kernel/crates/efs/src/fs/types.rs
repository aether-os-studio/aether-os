//! Definitions of needed types.
//!
//! See [the POSIX `<sys/types.h>` header](https://pubs.opengroup.org/onlinepubs/9699919799/basedefs/sys_types.h.html) for more information.

use core::cmp::Ordering;
use core::ops::{Add, AddAssign, Sub, SubAssign};
use core::time::Duration;

use derive_more::{Deref, DerefMut};

/// Used for device IDs.
///
/// It contains a [`u32`], following [the POSIX specification](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_types.h.html) and [the Linux implementation](https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tree/include/linux/types.h?h=linux-6.9.y#n21).
#[derive(Debug, Clone, Copy, Deref, DerefMut, Default)]
pub struct Dev(pub u32);

/// Used for file serial numbers.
///
/// It contains a [`usize`], following [the POSIX specification](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_types.h.html) and [the Linux implementation](https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tree/include/linux/types.h?h=linux-6.9.y#n22).
#[derive(Debug, Clone, Copy, Deref, DerefMut)]
pub struct Ino(pub u64);

/// Used for some file attributes.
///
/// It contains a [`u16`], following [the POSIX specification](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_types.h.html) and [the Linux implementation](https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tree/include/linux/types.h?h=linux-6.9.y#n23) ([`u16`] instead of `short` that doesn't exist in Rust to be compatible with 32-bits systems).
#[derive(Debug, Clone, Copy, Deref, DerefMut)]
pub struct Mode(pub u16);

/// Used for link counts.
///
/// It contains a [`u32`], following [the POSIX specification](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_types.h.html) and [the Linux implementation](https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tree/include/linux/types.h?h=linux-6.9.y#n25).
#[derive(Debug, Clone, Copy, Deref, DerefMut)]
pub struct Nlink(pub u32);

/// Used for user IDs.
///
/// It contains a [`u32`], following [the POSIX specification](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_types.h.html) and [the Linux implementation](https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tree/include/linux/types.h?h=linux-6.9.y#n37).
#[derive(Debug, Default, Clone, Copy, Deref, DerefMut)]
pub struct Uid(pub u32);

/// Used for group IDs.
///
/// It contains a [`u32`], following [the POSIX specification](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_types.h.html) and [the Linux implementation](https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tree/include/linux/types.h?h=linux-6.9.y#n38).
#[derive(Debug, Default, Clone, Copy, Deref, DerefMut)]
pub struct Gid(pub u32);

/// Used for file sizes.
///
/// It contains a [`isize`], following [the POSIX specification](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_types.h.html) and [the Linux implementation](https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tree/include/linux/types.h?h=linux-6.9.y#n26).
#[derive(Debug, Default, Clone, Copy, Deref, DerefMut)]
pub struct Off(pub isize);

/// Used for block sizes.
///
/// It contains a [`isize`], following [the POSIX specification](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_types.h.html) and [the Linux implementation](https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tree/tools/include/nolibc/std.h?h=linux-6.9.y#n32).
#[derive(Debug, Clone, Copy, Deref, DerefMut)]
pub struct Blksize(pub isize);

/// Used for file block counts.
///
/// It contains a [`i64`], following [the POSIX specification](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_types.h.html) and [the Linux implementation](https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tree/include/linux/types.h?h=linux-6.9.y#n26).
#[derive(Debug, Clone, Copy, Deref, DerefMut)]
pub struct Blkcnt(pub i64);

/// Used for time in seconds.
#[derive(
    Debug,
    Clone,
    Copy,
    Deref,
    DerefMut,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    derive_more::Add,
    derive_more::AddAssign,
    derive_more::Sub,
    derive_more::SubAssign,
)]
pub struct Time(pub i64);

impl Add<i64> for Time {
    type Output = Self;

    fn add(self, rhs: i64) -> Self::Output {
        Self(*self + rhs)
    }
}

impl AddAssign<i64> for Time {
    fn add_assign(&mut self, rhs: i64) {
        *self = *self + rhs;
    }
}

impl Sub<i64> for Time {
    type Output = Self;

    fn sub(self, rhs: i64) -> Self::Output {
        Self(*self - rhs)
    }
}

impl SubAssign<i64> for Time {
    fn sub_assign(&mut self, rhs: i64) {
        *self = *self - rhs;
    }
}

/// Used for precise instants.
///
/// Times shall be given in seconds since the Epoch. If possible, it can be completed with nanoseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timespec {
    /// Whole seconds.
    pub tv_sec: Time,

    /// Nanoseconds \[0, 999999999\].
    pub tv_nsec: u32,
}

impl PartialOrd for Timespec {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Timespec {
    /// Returns the classical [`Ordering`] between `self` and `other`.
    fn cmp(&self, other: &Self) -> Ordering {
        match self.tv_sec.cmp(&other.tv_sec) {
            Ordering::Less => Ordering::Less,
            Ordering::Equal => self.tv_nsec.cmp(&other.tv_nsec),
            Ordering::Greater => Ordering::Greater,
        }
    }
}

impl Add for Timespec {
    type Output = Self;

    /// Returns the sum of two [`Timespec`]: sums the seconds together, the nanoseconds together then take the
    /// [canonical](`Timespec::canonical`) value of the result.
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            tv_sec: self.tv_sec + rhs.tv_sec + (i64::from(self.tv_nsec) + i64::from(rhs.tv_nsec)) / 1_000_000_000,
            tv_nsec: (self.tv_nsec + rhs.tv_nsec) % 1_000_000_000,
        }
    }
}

impl Add<Time> for Timespec {
    type Output = Self;

    /// Adds the given time to the [`Timespec`] seconds.
    fn add(self, rhs: Time) -> Self::Output {
        self + Self {
            tv_sec: rhs,
            tv_nsec: 0,
        }
    }
}

impl AddAssign<Time> for Timespec {
    fn add_assign(&mut self, rhs: Time) {
        *self = *self + rhs;
    }
}

impl Sub for Timespec {
    type Output = Self;

    /// Returns the difference of two [`Timespec`]: substracts the seconds together, the nanoseconds together then take
    /// the [canonical](`Timespec::canonical`) value of the result to ensure that the nanoseconds are always positive.
    fn sub(self, rhs: Self) -> Self::Output {
        let self_nsec_gte_rhs_nsec = self.tv_nsec >= rhs.tv_nsec;
        Self {
            tv_sec: self.tv_sec - rhs.tv_sec - i64::from(!self_nsec_gte_rhs_nsec),
            tv_nsec: if self_nsec_gte_rhs_nsec {
                self.tv_nsec - rhs.tv_nsec
            } else {
                1_000_000_000 - (rhs.tv_nsec - self.tv_nsec)
            },
        }
    }
}

impl Sub<Time> for Timespec {
    type Output = Self;

    /// Substracts the given time to the [`Timespec`] seconds.
    fn sub(self, rhs: Time) -> Self::Output {
        self - Self {
            tv_sec: rhs,
            tv_nsec: 0,
        }
    }
}

impl SubAssign<Time> for Timespec {
    fn sub_assign(&mut self, rhs: Time) {
        *self = *self - rhs;
    }
}

impl From<Duration> for Timespec {
    fn from(value: Duration) -> Self {
        Self {
            tv_sec: Time(value.as_secs().try_into().expect("Cannot fit this duration on 63 bits")),
            // SAFETY: A positive integer under 1 000 000 000 will always fit on 32 bits.
            tv_nsec: unsafe { (u32::try_from(value.as_nanos() % 1_000_000_000)).unwrap_unchecked() },
        }
    }
}

#[cfg(feature = "std")]
#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
impl From<std::time::SystemTime> for Timespec {
    fn from(value: std::time::SystemTime) -> Self {
        value
            .duration_since(std::time::UNIX_EPOCH)
            .expect("The given system time is before the UNIX epoch")
            .into()
    }
}

impl Timespec {
    /// Returns the canonical form of the represented time.
    ///
    /// It ensures that the returned [`Timespec`] is such that the
    /// [`nanoseconds`](struct.Timespec.html#structfield.tv_nsec) are not greater or equal to `1_000_000_000`.
    #[must_use]
    pub fn canonical(self) -> Self {
        Self {
            tv_sec: self.tv_sec + (i64::from(self.tv_nsec) / 1_000_000_000),
            tv_nsec: self.tv_nsec % 1_000_000_000,
        }
    }

    #[allow(clippy::doc_link_with_quotes)]
    /// Returns the [`Timespec`] of ["now"](std::time::SystemTime::now).
    #[cfg(feature = "std")]
    #[cfg_attr(docsrs, doc(cfg(feature = "std")))]
    #[must_use]
    pub fn now() -> Self {
        std::time::SystemTime::now().into()
    }
}

#[cfg(test)]
mod test {
    use crate::fs::types::{Time, Timespec};

    #[test]
    fn time_ops_add() {
        assert_eq!(Time(100) + Time(100), Time(200));
        assert_eq!(Time(100) + 100, Time(200));

        let mut time = Time(100);
        time += Time(100);
        assert_eq!(time, Time(200));

        let mut time = Time(100);
        time += 100;
        assert_eq!(time, Time(200));
    }

    #[test]
    fn time_ops_sub() {
        assert_eq!(Time(200) - Time(100), Time(100));
        assert_eq!(Time(200) - 100, Time(100));

        let mut time = Time(100);
        time -= Time(200);
        assert_eq!(time, Time(-100));

        let mut time = Time(100);
        time -= 200;
        assert_eq!(time, Time(-100));
    }

    #[test]
    fn timespec_ops_add() {
        assert_eq!(
            Timespec {
                tv_sec: Time(100),
                tv_nsec: 12
            } + Timespec {
                tv_sec: Time(200),
                tv_nsec: 40
            },
            Timespec {
                tv_sec: Time(300),
                tv_nsec: 52
            }
        );
        assert_eq!(
            Timespec {
                tv_sec: Time(100),
                tv_nsec: 123
            } + Timespec {
                tv_sec: Time(200),
                tv_nsec: 999_999_998
            },
            Timespec {
                tv_sec: Time(301),
                tv_nsec: 121
            }
        );
        assert_eq!(
            Timespec {
                tv_sec: Time(100),
                tv_nsec: 12
            } + Time(200),
            Timespec {
                tv_sec: Time(300),
                tv_nsec: 12
            }
        );
    }

    #[test]
    fn timespec_ops_sub() {
        assert_eq!(
            Timespec {
                tv_sec: Time(100),
                tv_nsec: 12
            } - Timespec {
                tv_sec: Time(200),
                tv_nsec: 10
            },
            Timespec {
                tv_sec: Time(-100),
                tv_nsec: 2
            }
        );
        assert_eq!(
            Timespec {
                tv_sec: Time(200),
                tv_nsec: 123
            } - Timespec {
                tv_sec: Time(100),
                tv_nsec: 125
            },
            Timespec {
                tv_sec: Time(99),
                tv_nsec: 999_999_998
            }
        );
        assert_eq!(
            Timespec {
                tv_sec: Time(100),
                tv_nsec: 12
            } - Time(200),
            Timespec {
                tv_sec: Time(-100),
                tv_nsec: 12
            }
        );
    }
}
