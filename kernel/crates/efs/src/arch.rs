//! Architecture related functions, such as integer conversions.

use derive_more::Display;

use crate::error::Error;

/// Enumeration of possible errors encountered with integer convertion.
#[derive(Debug, Display)]
#[display("Arch Error: {_variant}")]
pub enum ArchError {
    /// Tried to convert a [`usize`] that does not fit into a [`u32`].
    #[display("usize to u32: {_0} cannot be converted into a u32")]
    UsizeToU32(usize),

    /// Tried to convert a [`u64`] that does not fit into a [`usize`].
    #[display("u64 to usize: {_0} cannot be converted into a usize")]
    U64ToUsize(u64),
}

impl core::error::Error for ArchError {}

/// Converts a [`usize`] into a [`u32`].
///
/// # Errors
///
/// Returns [`ArchError::UsizeToU32`] if the given [`usize`] does not fit on a [`u32`].
#[inline]
pub fn usize_to_u32(n: usize) -> Result<u32, Error<!>> {
    u32::try_from(n).map_err(|_| Error::Arch(ArchError::UsizeToU32(n)))
}

/// Converts a [`usize`] into a [`u64`].
#[inline]
#[must_use]
pub fn usize_to_u64(n: usize) -> u64 {
    // SAFETY: n is a u64 because usize <= u64 for all supported architectures
    unsafe { u64::try_from(n).unwrap_unchecked() }
}

/// Converts a [`u32`] into a [`usize`].
#[inline]
#[must_use]
pub fn u32_to_usize(#[allow(unused)] n: u32) -> usize {
    // SAFETY: n is a usize because u32 <= usize for all supported architectures
    unsafe { usize::try_from(n).unwrap_unchecked() }
}

/// Converts a [`u64`] into a [`usize`].
///
/// # Errors
///
/// Returns [`ArchError::U64ToUsize`] if the given [`u64`] does not fit on a [`usize`].
#[inline]
pub fn u64_to_usize(n: u64) -> Result<usize, Error<!>> {
    usize::try_from(n).map_err(|_| Error::Arch(ArchError::U64ToUsize(n)))
}
