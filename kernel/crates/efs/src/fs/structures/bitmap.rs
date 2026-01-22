//! Bitmap manipulation for devices.
//!
//! Bitmap are frequently used data types, so this is a general interface to manipulate them.

use alloc::vec::Vec;
use core::fmt::Debug;
use core::ops::{Deref, DerefMut};

use crate::arch::u32_to_usize;
use crate::celled::Celled;
use crate::dev::Device;
use crate::dev::address::Address;

/// Generic bitmap structure.
///
/// It can handles any [`Copy`] structure directly written onto a [`Device`].
///
/// See [the Wikipedia page](https://en.wikipedia.org/wiki/Bit_array) for more general informations.
pub struct Bitmap<Dev: Device> {
    /// Device containing the bitmap.
    device: Celled<Dev>,

    /// Inner elements.
    inner: Vec<u8>,

    /// Starting address of the bitmap on the device.
    starting_addr: Address,

    /// Length of the bitmap.
    length: u64,
}

impl<Dev: Device> Debug for Bitmap<Dev> {
    fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        fmt.debug_struct("Bitmap")
            .field("inner", &self.inner)
            .field("starting_addr", &self.starting_addr)
            .field("length", &self.length)
            .finish_non_exhaustive()
    }
}

impl<Dev: Device> Bitmap<Dev> {
    /// Creates a new [`Bitmap`] instance from the device on which it is located, its starting address on the device and
    /// its length.
    ///
    /// # Errors
    ///
    /// Returns a `no_std_io` [`Error`](deku::no_std_io::Error) if the device cannot be read.
    pub fn new(celled_device: Celled<Dev>, starting_addr: Address, length: u64) -> deku::no_std_io::Result<Self> {
        let inner = celled_device.lock().slice(starting_addr..(starting_addr + length))?.to_vec();
        Ok(Self {
            device: celled_device,
            inner,
            starting_addr,
            length,
        })
    }

    /// Returns the length of the bitmap.
    #[must_use]
    pub const fn length(&self) -> u64 {
        self.length
    }

    /// Returns the starting address of the bitmap.
    #[must_use]
    pub const fn starting_address(&self) -> Address {
        self.starting_addr
    }

    /// Writes back the current state of the bitmap onto the device.
    ///
    /// # Errors
    ///
    /// Returns a `no_std_io` [`Error`](deku::no_std_io::Error) if the device cannot be written.
    pub fn write_back(&mut self) -> deku::no_std_io::Result<()> {
        let mut device = self.device.lock();
        let mut slice = device.slice(self.starting_addr..(self.starting_addr + self.length))?;
        slice.clone_from_slice(&self.inner);
        let commit = slice.commit();
        device.commit(commit)?;

        Ok(())
    }

    /// Finds the first elements `el` such that the sum of all `count(el)` is greater than or equal to `n`.
    ///
    /// Returns the indices and the value of those elements, keeping only the ones satisfying `count(el) > 0`.
    ///
    /// If the sum of all `count(el)` is lesser than `n`, returns all the elements `el` such that `count(el) > 0`.
    pub fn find_to_count<F: Fn(&u8) -> usize>(&self, n: usize, count: F) -> Vec<(usize, u8)> {
        let mut counter = 0_usize;
        let mut element_taken = Vec::new();

        for (index, element) in self.inner.iter().enumerate() {
            let element_count = count(element);
            if element_count > 0 {
                counter += element_count;
                element_taken.push((index, *element));
                if counter >= n {
                    return element_taken;
                }
            }
        }

        element_taken
    }

    /// Specialization of [`find_to_count`](Bitmap::find_to_count) to find the first bytes such that the sum of set bits
    /// is at least `n`.
    #[must_use]
    pub fn find_n_set_bits(&self, n: usize) -> Vec<(usize, u8)> {
        self.find_to_count(n, |byte| {
            let mut count = byte - ((byte >> 1_u8) & 0x55);
            count = (count & 0x33) + ((count >> 2_u8) & 0x33);
            count = (count + (count >> 4_u8)) & 0x0F;
            u32_to_usize(count.into())
        })
    }

    /// Specialization of [`find_to_count`](Bitmap::find_to_count) to find the first bytes such that the sum of unset
    /// bits is at least `n`.
    #[must_use]
    pub fn find_n_unset_bits(&self, n: usize) -> Vec<(usize, u8)> {
        self.find_to_count(n, |byte| {
            let mut count = byte - ((byte >> 1_u8) & 0x55);
            count = (count & 0x33) + ((count >> 2_u8) & 0x33);
            count = (count + (count >> 4_u8)) & 0x0F;
            u32_to_usize(8_u32 - Into::<u32>::into(count))
        })
    }
}

impl<Dev: Device> IntoIterator for Bitmap<Dev> {
    type IntoIter = <Vec<u8> as IntoIterator>::IntoIter;
    type Item = u8;

    fn into_iter(self) -> Self::IntoIter {
        self.inner.into_iter()
    }
}

impl<Dev: Device> Deref for Bitmap<Dev> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<Dev: Device> DerefMut for Bitmap<Dev> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}
