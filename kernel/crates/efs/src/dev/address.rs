//! General description of addresses in a device.

use core::fmt::Debug;
use core::iter::Step;
use core::ops::Mul;

use derive_more::{Add, Deref, DerefMut, LowerHex, Sub};

use crate::arch::usize_to_u64;

/// Address of a physical sector
#[derive(Debug, Clone, Copy, PartialEq, Eq, LowerHex, PartialOrd, Ord, Deref, DerefMut, Add, Sub)]
pub struct Address(u64);

impl Address {
    /// Returns a new [`Address`] from its index.
    ///
    /// This function is equivalent to the [`From<u64>`](struct.Address.html#impl-From<u64>-for-Address)
    /// implementation but with a `const fn`.
    #[must_use]
    pub const fn new(index: u64) -> Self {
        Self(index)
    }

    /// Returns the index of this address, which corresponds to its offset from the start of the device.
    #[must_use]
    pub const fn index(&self) -> u64 {
        self.0
    }
}

impl From<u64> for Address {
    fn from(index: u64) -> Self {
        Self(index)
    }
}

impl From<Address> for u64 {
    fn from(value: Address) -> Self {
        value.0
    }
}

impl From<u32> for Address {
    fn from(value: u32) -> Self {
        Self(u64::from(value))
    }
}

impl From<usize> for Address {
    fn from(value: usize) -> Self {
        usize_to_u64(value).into()
    }
}

impl const core::ops::Add<u64> for Address {
    type Output = Self;

    fn add(self, rhs: u64) -> Self::Output {
        Self(self.0 + rhs)
    }
}

impl core::ops::Sub<u64> for Address {
    type Output = Self;

    fn sub(self, rhs: u64) -> Self::Output {
        Self(*self - rhs)
    }
}

impl Mul<u64> for Address {
    type Output = Self;

    fn mul(self, rhs: u64) -> Self::Output {
        Self(*self * rhs)
    }
}

impl Step for Address {
    fn steps_between(start: &Self, end: &Self) -> (usize, Option<usize>) {
        u64::steps_between(start, end)
    }

    fn forward_checked(start: Self, count: usize) -> Option<Self> {
        u64::forward_checked(*start, count).map(Into::into)
    }

    fn backward_checked(start: Self, count: usize) -> Option<Self> {
        u64::backward_checked(*start, count).map(Into::into)
    }
}
