//! Generic implementation of blocks in a filesystem.
//!
//! A block is defined here as a contiguous split of the [`Device`] containing the
//! [`Filesystem`](crate::fs::Filesystem). All blocks have the same size in bytes. This is **NOT** a block as in block
//! device, here "block" always refers to the filesystem's block. They start at 0, so the `n`th block will start at the
//! address `n * block_size`. Thus, a block is entirely described by its number.

use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};

use deku::no_std_io::{Read, Seek, SeekFrom, Write};

use crate::arch::{u32_to_usize, usize_to_u64};
use crate::celled::Celled;
use crate::dev::Device;
use crate::dev::address::Address;

/// A generic block.
///
/// It is a part of a [`Device`] with a fixed size (usually determined by the super-block of the filesystem). They
/// numbering starts at 0, so the `n`th block will start at the address `n * block_size`.
pub trait Block<Dev: Device> {
    /// Type used to number blocks in the filestem.
    type Num: Into<u64>;

    /// Returns the size of a block.
    ///
    /// This function should always return the same value for a given filesystem as a block size should not change
    /// during a normal manipulation of a filesystem.
    fn size(&self) -> u32;

    /// Returns the current block number.
    fn number(&self) -> Self::Num;

    /// Returns the device containing the block.
    ///
    /// This function may seem weird, but it allows to make a very generic implementation of [`Block`]s.
    fn device(&mut self) -> Celled<Dev>;
}

/// Wrapper around the [`Block`] trait to provide [`no_std_io`](deku::no_std_io) traits implementation.
pub struct BlockWrapper<Dev: Device, B: Block<Dev>> {
    /// Inner block.
    inner: B,

    /// Offset for the I/O operations.
    io_offset: u64,

    /// Phantom data for the file system error and the device.
    phantom: PhantomData<Dev>,
}

impl<Dev: Device, B: Block<Dev>> const Deref for BlockWrapper<Dev, B> {
    type Target = B;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<Dev: Device, B: Block<Dev>> const DerefMut for BlockWrapper<Dev, B> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<Dev: Device, B: Block<Dev>> Read for BlockWrapper<Dev, B> {
    fn read(&mut self, buf: &mut [u8]) -> deku::no_std_io::Result<usize> {
        let binder = self.device();
        let mut device = binder.lock();

        let length = (u32_to_usize(self.inner.size())
            // SAFETY: self.io_offset & usize_to_u64(usize::MAX) always fit on a usize
            - unsafe { usize::try_from(self.io_offset & usize_to_u64(usize::MAX)).unwrap_unchecked() })
        .min(buf.len());
        let starting_addr =
            Address::new(Into::<u64>::into(self.number()) * u64::from(self.inner.size()) + self.io_offset);

        let slice = device.slice(starting_addr..starting_addr + usize_to_u64(length))?;
        buf.clone_from_slice(slice.as_ref());

        self.io_offset += usize_to_u64(length);

        Ok(length)
    }
}

impl<Dev: Device, B: Block<Dev>> Write for BlockWrapper<Dev, B> {
    fn write(&mut self, buf: &[u8]) -> deku::no_std_io::Result<usize> {
        let binder = self.device();
        let mut device = binder.lock();

        let length = (u32_to_usize(self.inner.size())
            // SAFETY: self.io_offset & usize_to_u64(usize::MAX) always fit on a usize
            - unsafe { usize::try_from(self.io_offset & usize_to_u64(usize::MAX)).unwrap_unchecked() })
        .min(buf.len());
        let starting_addr =
            Address::new(Into::<u64>::into(self.number()) * u64::from(self.inner.size()) + self.io_offset);
        let mut slice = device.slice(starting_addr..starting_addr + usize_to_u64(length))?;

        // SAFETY: buf size is at least length
        slice.clone_from_slice(unsafe { buf.get_unchecked(..length) });
        let commit = slice.commit();
        device.commit(commit)?;

        self.io_offset += usize_to_u64(length);

        Ok(length)
    }

    fn flush(&mut self) -> deku::no_std_io::Result<()> {
        Ok(())
    }
}

impl<Dev: Device, B: Block<Dev>> Seek for BlockWrapper<Dev, B> {
    fn seek(&mut self, pos: SeekFrom) -> deku::no_std_io::Result<u64> {
        let block_size = i64::from(self.inner.size());
        let previous_offset = self.io_offset;
        match pos {
            SeekFrom::Start(offset) => self.io_offset = offset,
            SeekFrom::End(back_offset) => {
                self.io_offset = u64::try_from(block_size + back_offset).map_err(|_err| {
                    deku::no_std_io::Error::new(
                        deku::no_std_io::ErrorKind::InvalidInput,
                        "Invalid seek to a negative or overflowing position",
                    )
                })?;
            },
            SeekFrom::Current(add_offset) => {
                self.io_offset = (i64::try_from(previous_offset)
                    .expect("The I/O offset of a block is in an incoherent state")
                    + add_offset)
                    .try_into()
                    .map_err(|_err| {
                        deku::no_std_io::Error::new(
                            deku::no_std_io::ErrorKind::InvalidInput,
                            "Invalid seek to a negative or overflowing position",
                        )
                    })?;
            },
        }

        if self.io_offset >= u64::from(self.inner.size()) {
            Err(deku::no_std_io::Error::new(
                deku::no_std_io::ErrorKind::InvalidInput,
                "Invalid seek to a negative or overflowing position",
            ))
        } else {
            Ok(previous_offset)
        }
    }
}

impl<Dev: Device, B: Block<Dev>> BlockWrapper<Dev, B> {
    /// Returns a [`BlockWrapper`] from its inner [`Block`].
    pub const fn new_wrapper(block: B) -> Self {
        Self {
            inner: block,
            io_offset: 0,
            phantom: PhantomData,
        }
    }
}
