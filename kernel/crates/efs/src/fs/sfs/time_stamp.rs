//! Implementation of the [time stamps](../index.html#time-stamps) of SFS.
//!
//! It is closely linked to the POSIX paths, whose implementation is available [in this
//! crate](crate::fs::types::Timespec).
//!
//! See the [official documentation](https://web.archive.org/web/20170315134201/https://www.d-rift.nl/combuster/vdisk/sfs.html#Time_Stamps) for more information.

use core::ops::{Add, AddAssign, Sub, SubAssign};

use derive_more::derive::Deref;

use super::error::SfsError;
use crate::error::Error;
use crate::fs::error::FsError;
use crate::fs::types::{Time, Timespec};

/// A SFS time stamp.
///
/// It is a signed 64-bits value that represent the number of 1/65536ths of a second since the beginning of the 1st of
/// January 1970.
///
/// For example, the value `0x00000000003C0000` would represent one minute past midnight on the 1st of January 1970,
/// while the value `0x0000000000000001` would represent roughly 15259 ns past midnight on the 1st of January 1970.
///
/// All time stamps are in UTC (Universal Co-ordinated Time) so that problems with time zones and daylight savings are
/// avoided.
#[derive(
    Debug,
    Clone,
    Copy,
    Deref,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    derive_more::derive::Add,
    derive_more::derive::AddAssign,
    derive_more::derive::Sub,
    derive_more::derive::SubAssign,
)]
pub struct TimeStamp(i64);

impl From<i64> for TimeStamp {
    fn from(value: i64) -> Self {
        Self(value)
    }
}

impl From<TimeStamp> for i64 {
    fn from(value: TimeStamp) -> Self {
        value.0
    }
}

impl Add<i64> for TimeStamp {
    type Output = Self;

    fn add(self, rhs: i64) -> Self::Output {
        Self(*self + rhs)
    }
}

impl AddAssign<i64> for TimeStamp {
    fn add_assign(&mut self, rhs: i64) {
        *self = *self + rhs;
    }
}

impl Sub<i64> for TimeStamp {
    type Output = Self;

    fn sub(self, rhs: i64) -> Self::Output {
        Self(*self - rhs)
    }
}

impl SubAssign<i64> for TimeStamp {
    fn sub_assign(&mut self, rhs: i64) {
        *self = *self - rhs;
    }
}

impl From<TimeStamp> for Timespec {
    fn from(value: TimeStamp) -> Self {
        // Proportionality table:
        // +--------------------+----------+
        // |      1/2^16 s      |   1 ns   | -> unit
        // +--------------------+----------+
        // | `value.0 % 65536`  |     ?    |
        // +--------------------+----------+
        // |        2^16        |   10^9   | -> 1 second
        // +--------------------+----------+
        Self {
            tv_sec: Time(value.0 / 65536),
            // SAFETY: The maximal value of this calcul is 999 984 741, which fits on 32-bits
            tv_nsec: unsafe { u32::try_from(((value.0.abs() % 65536) * 1_000_000_000) / 65536).unwrap_unchecked() },
        }
    }
}

impl TryFrom<Timespec> for TimeStamp {
    type Error = Error<SfsError>;

    fn try_from(value: Timespec) -> Result<Self, Self::Error> {
        // Proportionality table:
        // +------------+------------------+
        // |  1/2^16 s  |       1 ns       | -> unit
        // +------------+------------------+
        // |      ?     |  `value.tv_nsec` |
        // +------------+------------------+
        // |    2^16    |       10^9       | -> 1 second
        // +------------+------------------+
        Ok(Self(
            value
                .tv_sec
                .0
                .checked_mul(65536)
                .and_then(|res| res.checked_add((i64::from(value.tv_nsec) * 65536) / 1_000_000_000))
                .ok_or_else(|| Error::Fs(FsError::Implementation(SfsError::TimeStampOutOfBounds(value))))?,
        ))
    }
}

#[cfg(feature = "std")]
#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
impl TryFrom<std::time::SystemTime> for TimeStamp {
    type Error = Error<SfsError>;

    fn try_from(value: std::time::SystemTime) -> Result<Self, Self::Error> {
        Timespec::from(value).try_into()
    }
}

impl TimeStamp {
    #[allow(clippy::doc_link_with_quotes)]
    /// Returns the [`TimeStamp`] of ["now"](std::time::SystemTime::now).
    ///
    /// # Errors
    ///
    /// Returns [`SfsError::TimeStampOutOfBounds`] if the current time, given in seconds since the
    /// [`UNIX_EPOCH`](std::time::UNIX_EPOCH), cannot fit on 47 bits.
    #[cfg(feature = "std")]
    #[cfg_attr(docsrs, doc(cfg(feature = "std")))]
    pub fn now() -> Result<Self, Error<SfsError>> {
        std::time::SystemTime::now().try_into()
    }
}

#[cfg(test)]
mod test {
    use crate::fs::sfs::time_stamp::TimeStamp;
    use crate::fs::types::{Time, Timespec};

    #[test]
    fn i64_conversion() {
        assert_eq!(0_i64, TimeStamp::from(0_i64).into());
        assert_eq!(1_000_000_i64, TimeStamp::from(1_000_000_i64).into());
        assert_eq!(-1_000_000_i64, TimeStamp::from(-1_000_000_i64).into());
    }

    #[test]
    fn time_conversion() {
        const YEAR_2000_TIMESTAMP: i64 = 946_684_800;

        assert_eq!(
            TimeStamp::from(YEAR_2000_TIMESTAMP * 65536),
            (Timespec {
                tv_sec: Time(YEAR_2000_TIMESTAMP),
                tv_nsec: 0
            })
            .try_into()
            .unwrap()
        );
        assert_eq!(
            TimeStamp::from(YEAR_2000_TIMESTAMP * 65536 + 768),
            (Timespec {
                tv_sec: Time(YEAR_2000_TIMESTAMP),
                tv_nsec: u32::try_from((768 * 1_000_000_000_u64) / 65536_u64).unwrap()
            })
            .try_into()
            .unwrap()
        );

        assert_eq!(
            Timespec {
                tv_sec: Time(YEAR_2000_TIMESTAMP),
                tv_nsec: 0
            },
            TimeStamp::from(YEAR_2000_TIMESTAMP * 65536).into()
        );
        assert_eq!(
            Timespec {
                tv_sec: Time(YEAR_2000_TIMESTAMP),
                // Round error -> TimeStamp can only be precise up to 1/65536th of a second, which is about ~15258ns
                tv_nsec: 123_456 - 1386
            },
            TimeStamp::from(YEAR_2000_TIMESTAMP * 65536 + (123_456 * 65536) / 1_000_000_000).into()
        );
    }

    #[test]
    fn now_conversion() {
        let timestamp_now = TimeStamp::now().unwrap();
        let timespec_now = Timespec::now();
        assert!(
            timestamp_now <= TimeStamp::try_from(timespec_now).unwrap() + 1
                && TimeStamp::try_from(timespec_now).unwrap() <= timestamp_now + 1
        );
        assert!(
            (Timespec::now()
                - Timespec {
                    tv_sec: Time(0),
                    tv_nsec: 15259
                })
                <= Timespec::from(TimeStamp::now().unwrap())
                && Timespec::from(TimeStamp::now().unwrap()) <= Timespec::now() + Time(1)
        );
    }
}
