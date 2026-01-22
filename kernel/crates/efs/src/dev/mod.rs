//! Everything related to the devices.
//!
//! # Devices
//!
//! In this crate, a [`Device`] is a structure capable of storing a fixed number of contiguous bytes. Each byte is
//! located at a unique [`Address`], which is used to describe [`Slice`]s. When a [`Slice`] is mutated, it can be
//! committed into a [`Commit`], which can be write back on the [`Device`].
//!
//! To avoid manipulating only [`Slice`]s and [`Commit`]s, which is quite heavy, you can manipulate a [`Device`] through
//! the provided metehods [`read_from_bytes`](Device::read_from_bytes) and [`write_to_bytes`](Device::write_to_bytes).
//!
//! If needed for the [`Filesystem`](crate::fs::Filesystem) usage, the device may be able to give the current time.
//!
//! ## How to implement a device?
//!
//! ### Derived automatically
//!
//! The easiest way for an object to implement [`Device`] is to make it implements the traits [`Read`], [`Write`] and
//! [`Seek`] from [`deku::no_std_io`] or [`no_std_io2`](https://crates.io/crates/no_std_io2) (which is the same crate
//! re-exported). The advantage of such requirements is that in a [`std`] environment, those traits are exactly the
//! ones of [`std::io`], which is helpful not to define twice the same functions.
//!
//! ```
//! use std::error::Error;
//! use std::fmt::Display;
//!
//! use deku::DekuRead;
//! use deku::no_std_io::{Read, Seek, SeekFrom, Write};
//! use efs::dev::Device;
//! use efs::dev::address::Address;
//! use efs::fs::file::Base;
//!
//! struct Foo {} // Foo is our device
//!
//! impl Read for Foo {
//!     fn read(&mut self, buf: &mut [u8]) -> deku::no_std_io::Result<usize> {
//!         buf.fill(1);
//!         Ok(buf.len())
//!     }
//! }
//!
//! impl Write for Foo {
//!     fn write(&mut self, buf: &[u8]) -> deku::no_std_io::Result<usize> {
//!         Ok(buf.len())
//!     }
//!
//!     fn flush(&mut self) -> deku::no_std_io::Result<()> {
//!         Ok(())
//!     }
//! }
//!
//! impl Seek for Foo {
//!     fn seek(&mut self, _pos: SeekFrom) -> deku::no_std_io::Result<u64> {
//!         Ok(0)
//!     }
//! }
//!
//! #[derive(Debug, PartialEq, Eq, DekuRead)]
//! struct Bar {
//!     a: u8,
//!     b: u32,
//! }
//!
//! let mut foo = Foo {};
//!
//! // Now `foo` implements `Device,
//! // thus we can use all the methods from the `Device` trait.
//!
//! assert_eq!(foo.read_from_bytes::<Bar>(Address::new(0), 5).unwrap(), Bar {
//!     a: 0x1,
//!     b: 0x0101_0101
//! });
//! ```
//!
//! Moreover, when using the `std` feature, the devices derived automatically will use [`std_now`] for the
//! implementation of the [`Device::now`] method.
//!
//! ### By hand
//!
//! To implement a device, you need to provide three methods:
//!
//! * [`size`](Device::size) which returns the size of the device in bytes
//!
//! * [`slice`](Device::slice) which creates a [`Slice`] of the device
//!
//! * [`commit`](Device::commit) which commits a [`Commit`] created from a mutated [`Slice`] of the device
//!
//! * [`now`](Device::now) (optional) which returns the current time.
//!
//! To help you, here is an example of how those methods can be used:
//!
//! ```
//! use std::vec;
//!
//! use efs::dev::address::Address;
//! use efs::dev::{Device, Wrapper};
//!
//! // Here, our device is a `Wrapper<Vec<usize>>`
//! let mut device = Wrapper::new(vec![0_u8; 1024]);
//!
//! // We take a slice of the device: `slice` now contains a reference to the
//! // objects between the indices 256 (included) and 512 (not included) of the
//! // device.
//! let mut slice = device.slice(Address::from(256_u64)..Address::from(512_u64)).unwrap();
//!
//! // We modify change each elements `0` to a `1` in the slice.
//! slice.iter_mut().for_each(|element| *element = 1);
//!
//! // We commit the changes of slice: now this slice cannot be changed anymore.
//! let commit = slice.commit();
//!
//! assert!(device.commit(commit).is_ok());
//!
//! for (idx, &x) in device.iter().enumerate() {
//!     assert_eq!(x, u8::from((256..512).contains(&idx)));
//! }
//! ```
//!
//! Moreover, your implementation of a device should only returns [`deku::no_std_io::Error`] error in case of a
//! read/write fail.

use alloc::borrow::{Cow, ToOwned};
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::iter::Step;
use core::mem::{size_of, transmute_copy};
use core::ops::{Deref, DerefMut, Range};
use core::ptr::{addr_of, slice_from_raw_parts};

use deku::no_std_io::{Read, Seek, SeekFrom, Write};
use deku::{DekuContainerRead, DekuContainerWrite};
use derive_more::{Constructor, Deref, DerefMut};

use self::address::Address;
use self::size::Size;
use crate::arch::usize_to_u64;
#[cfg(feature = "std")]
use crate::fs::types::Time;
use crate::fs::types::Timespec;

pub mod address;
pub mod size;

/// Slice of a device, filled with objects of type `T`.
#[derive(Debug, Clone)]
pub struct Slice<'mem> {
    /// Elements of the slice.
    inner: Cow<'mem, [u8]>,

    /// Starting address of the slice.
    starting_addr: Address,
}

impl AsRef<[u8]> for Slice<'_> {
    fn as_ref(&self) -> &[u8] {
        &self.inner
    }
}

impl AsMut<[u8]> for Slice<'_> {
    fn as_mut(&mut self) -> &mut [u8] {
        self.inner.to_mut().as_mut()
    }
}

impl Deref for Slice<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.as_ref()
    }
}

impl DerefMut for Slice<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut()
    }
}

impl<'mem> Slice<'mem> {
    /// Creates a new [`Slice`].
    #[must_use]
    pub const fn new(inner: &'mem [u8], starting_addr: Address) -> Self {
        Self {
            inner: Cow::Borrowed(inner),
            starting_addr,
        }
    }

    /// Creates a new [`Slice`] from [`ToOwned::Owned`] objects.
    #[must_use]
    pub const fn new_owned(inner: <[u8] as ToOwned>::Owned, starting_addr: Address) -> Self {
        Self {
            inner: Cow::Owned(inner),
            starting_addr,
        }
    }

    /// Returns the starting address of the slice.
    #[must_use]
    pub const fn addr(&self) -> Address {
        self.starting_addr
    }

    /// Checks whether the slice has been mutated or not.
    #[must_use]
    pub const fn is_mutated(&self) -> bool {
        match self.inner {
            Cow::Borrowed(_) => false,
            Cow::Owned(_) => true,
        }
    }

    /// Commits the write operations onto the slice and returns a [`Commit`]ed object.
    #[must_use]
    pub fn commit(self) -> Commit {
        Commit::new(self.inner.into_owned(), self.starting_addr)
    }
}

impl<'mem> Slice<'mem> {
    /// Returns the content of this slice as an object `T`.
    ///
    /// # Safety
    ///
    /// Must ensure that an instance of `T` is located at the begining of the slice and that the length of the slice is
    /// greater than the memory size of `T`.
    ///
    /// # Panics
    ///
    /// Panics if the starting address cannot be read.
    #[must_use]
    pub unsafe fn cast<T: Copy>(&self) -> T {
        assert!(
            self.inner.len() >= size_of::<T>(),
            "The length of the device slice is not great enough to contain an object T"
        );
        unsafe { transmute_copy(self.inner.as_ptr().as_ref().expect("Could not read the pointer of the slice")) }
    }

    /// Creates a [`Slice`] from any [`Copy`] object.
    pub const fn from<T: Copy>(object: T, starting_addr: Address) -> Self {
        let len = size_of::<T>();
        let ptr = addr_of!(object).cast::<u8>();
        // SAFETY: the pointer is well-formed since it has been created above
        let inner_opt = unsafe { slice_from_raw_parts(ptr, len).as_ref::<'mem>() };
        // SAFETY: `inner_opt` cannot be `None` as `ptr` contains data and the call to `slice_from_raw_parts` should not
        // return a null pointer
        Self::new(unsafe { inner_opt.unwrap_unchecked() }, starting_addr)
    }
}

/// Commited slice of a device, filled with objects of type `T`.
#[derive(Debug, Clone)]
pub struct Commit {
    /// Elements of the commit.
    inner: Vec<u8>,

    /// Starting address of the slice.
    starting_addr: Address,
}

impl Commit {
    /// Creates a new [`Commit`] instance.
    #[must_use]
    pub const fn new(inner: Vec<u8>, starting_addr: Address) -> Self {
        Self {
            inner,
            starting_addr,
        }
    }

    /// Returns the starting address of the commit.
    #[must_use]
    pub const fn addr(&self) -> Address {
        self.starting_addr
    }
}

impl AsRef<[u8]> for Commit {
    fn as_ref(&self) -> &[u8] {
        &self.inner
    }
}

impl AsMut<[u8]> for Commit {
    fn as_mut(&mut self) -> &mut [u8] {
        self.inner.as_mut()
    }
}

/// General interface for devices containing a file system.
pub trait Device {
    /// [`Size`] description of this device (in bytes).
    fn size(&mut self) -> Size;

    /// Returns a [`Slice`] with elements of this device.
    ///
    /// Must ensure that the elements got are exactly the one described by `addr_range`.
    ///
    /// # Errors
    ///
    /// Returns an [`Error`](deku::no_std_io::Error) if the read could not be completed.
    fn slice(&mut self, addr_range: Range<Address>) -> deku::no_std_io::Result<Slice<'_>>;

    /// Writes the [`Commit`] onto the device.
    ///
    /// # Errors
    ///
    /// Returns an [`Error`](deku::no_std_io::Error) if the write could not be completed.
    fn commit(&mut self, commit: Commit) -> deku::no_std_io::Result<()>;

    /// Reads an element of type `O` on the device starting at the address `starting_addr`.
    ///
    /// The element must be entirely contained in **at most** `length` bytes, and the device **must** contain at least
    /// `length` bytes after `starting_addr`.
    ///
    /// # Errors
    ///
    /// Returns an [`ErrorKind::InvalidInput`](deku::no_std_io::ErrorKind::InvalidInput) if the read tries to go out of
    /// the device's bounds or if [`Device::slice`] failed.
    fn read_from_bytes<O: for<'a> DekuContainerRead<'a>>(
        &mut self,
        starting_addr: Address,
        length: usize,
    ) -> deku::no_std_io::Result<O> {
        let range = starting_addr..Address::forward_checked(starting_addr, length).ok_or_else(|| {
            deku::no_std_io::Error::new(deku::no_std_io::ErrorKind::InvalidInput, "Tried to reach an invalid address")
        })?;
        let slice = self.slice(range)?;
        O::from_bytes((&slice, 0)).map(|(_, obj)| obj).map_err(Into::into)
    }

    /// Writes an element of type `O` on the device starting at the address `starting_addr`.
    ///
    /// Beware, the `object` **must be the owned `O` object and not a borrow**, otherwise the pointer to the object will
    /// be copied, and not the object itself.
    ///
    /// # Errors
    ///
    /// Returns an [`ErrorKind::InvalidInput`](deku::no_std_io::ErrorKind::InvalidInput) if the read tries to go out of
    /// the device's bounds or if [`Device::slice`] or [`Device::commit`] failed.
    fn write_to_bytes<O: DekuContainerWrite>(&mut self, starting_addr: Address, obj: O) -> deku::no_std_io::Result<()> {
        let obj_bytes = obj.to_bytes()?;
        let length = obj_bytes.len();
        let range = starting_addr..Address::forward_checked(starting_addr, length).ok_or_else(|| {
            deku::no_std_io::Error::new(deku::no_std_io::ErrorKind::InvalidInput, "Tried to reach an invalid address")
        })?;
        let mut device_slice = self.slice(range)?;

        let buffer = device_slice
            .get_mut(..)
            .unwrap_or_else(|| unreachable!("It is always possible to take all the elements of a slice"));

        buffer.copy_from_slice(&obj_bytes);

        let commit = device_slice.commit();
        self.commit(commit)
    }

    /// Returns the current [`Timespec`] if the device is able to.
    ///
    /// Otherwise, returns [`None`].
    fn now(&mut self) -> Option<Timespec> {
        None
    }
}

/// Returns the current time in the [`Timespec`] format.
///
/// Gives a direct implementation of [`Device::now`] for `std` devices.
#[cfg(feature = "std")]
#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
#[must_use]
pub fn std_now() -> Timespec {
    // SAFETY: UNIX_EPOCH was the 01/01/1970 (midnight UTC/GMT), so "now" will always be after this instant
    let now = unsafe { std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_unchecked() };
    Timespec {
        // SAFETY: the number of seconds from the UNIX_EPOCH will reach i64::MAX in billions of years
        tv_sec: Time(unsafe { now.as_secs().try_into().unwrap_unchecked() }),
        // SAFETY: 1_000_000_000 can always be converted into a u32
        tv_nsec: unsafe { u32::try_from(now.as_nanos() % 1_000_000_000).unwrap_unchecked() },
    }
}

impl<T: Read + Write + Seek> Device for T {
    fn size(&mut self) -> Size {
        let offset = self.seek(SeekFrom::End(0)).expect("Could not seek the device at its end");
        let size = self
            .seek(SeekFrom::Start(offset))
            .expect("Could not seek the device at its original offset");
        Size(size)
    }

    fn slice(&mut self, addr_range: Range<Address>) -> deku::no_std_io::Result<Slice<'_>> {
        let starting_addr = addr_range.start;
        let len = TryInto::<usize>::try_into((addr_range.end - addr_range.start).index()).map_err(|_err| {
            deku::no_std_io::Error::new(deku::no_std_io::ErrorKind::InvalidInput, "Tried to reach an invalid address")
        })?;

        let mut slice = alloc::vec![0; len];
        self.seek(SeekFrom::Start(starting_addr.index()))?;
        self.read_exact(&mut slice)?;

        Ok(Slice::new_owned(slice, starting_addr))
    }

    fn commit(&mut self, commit: Commit) -> deku::no_std_io::Result<()> {
        let offset = self.seek(SeekFrom::Start(commit.addr().index()))?;
        self.write_all(commit.as_ref())?;
        self.seek(SeekFrom::Start(offset))?;

        Ok(())
    }

    #[cfg(feature = "std")]
    fn now(&mut self) -> Option<Timespec> {
        Some(std_now())
    }
}

/// Wrapper structure to be able to use [`Vec`], [`Box`] and [`array`] as devices.
///
/// This structure is primarily intended for testing purposes.
#[derive(Debug, Clone, Copy, Constructor, Default, Deref, DerefMut)]
pub struct Wrapper<T>(T);

/// Generic implementation of the [`Device`] trait.
macro_rules! impl_device {
    ($volume:ty) => {
        impl Device for Wrapper<$volume> {
            fn size(&mut self) -> Size {
                Size(usize_to_u64(self.len()))
            }

            fn slice(&mut self, addr_range: Range<Address>) -> deku::no_std_io::Result<Slice<'_>> {
                if Device::size(self) >= u64::from(addr_range.end) {
                    let addr_start = addr_range.start;
                    let range = usize::try_from(addr_range.start.index()).expect(
                        "Unreachable: tried to handle a structure that need more RAM that the system can handle",
                    )
                        ..usize::try_from(addr_range.end.index()).expect(
                            "Unreachable: tried to handle a structure that need more RAM that the system can handle",
                        );
                    // SAFETY: it is checked above that the wanted elements exist
                    Ok(Slice::new(unsafe { <$volume as AsRef<[u8]>>::as_ref(self).get_unchecked(range) }, addr_start))
                } else {
                    Err(

            deku::no_std_io::Error::new(deku::no_std_io::ErrorKind::InvalidInput, "Tried to reach an invalid address")
                    )
                }
            }

            fn commit(&mut self, commit: Commit) -> deku::no_std_io::Result<()> {
                let addr_start = commit.addr().index();
                let addr_end = addr_start + usize_to_u64(commit.as_ref().len());

                let dest = &mut <$volume as AsMut<[u8]>>::as_mut(self).get_mut(usize::try_from(addr_start).expect(
                    "Unreachable: tried to handle a structure that need more RAM that the system can handle",
                )
                    ..usize::try_from(addr_end).expect(
                        "Unreachable: tried to handle a structure that need more RAM that the system can handle",
                    )).ok_or_else(|| {
                    deku::no_std_io::Error::new(deku::no_std_io::ErrorKind::InvalidInput, "Tried to reach an invalid address")
                })?;
                dest.clone_from_slice(&commit.as_ref());
                Ok(())
            }
        }
    };
}

impl_device!(&mut [u8]);
impl_device!(Vec<u8>);
impl_device!(Box<[u8]>);

#[cfg(test)]
mod test {
    use alloc::string::String;
    use alloc::vec;
    use std::fs::{self, File};

    use deku::no_std_io::{Read, Seek, SeekFrom, Write};
    use deku::{DekuContainerWrite, DekuRead, DekuWrite};

    use crate::dev::address::Address;
    use crate::dev::{Device, Wrapper};

    #[test]
    fn device_generic_read() {
        let mut device = Wrapper::new(vec![0_u8; 1024]);
        let mut slice = device.slice(Address::from(256_u32)..Address::from(512_u32)).unwrap();
        slice.iter_mut().for_each(|element| *element = 1);

        let commit = slice.commit();

        assert!(device.commit(commit).is_ok());

        for (idx, &x) in device.iter().enumerate() {
            assert_eq!(x, u8::from((256..512).contains(&idx)));
        }
    }

    #[allow(clippy::missing_asserts_for_indexing)]
    fn device_file_write(mut file_1: File) {
        let mut slice = file_1.slice(Address::new(0)..Address::new(13)).unwrap();

        let word = slice.get_mut(6..=10).unwrap();
        word[0] = b'e';
        word[1] = b'a';
        word[2] = b'r';
        word[3] = b't';
        word[4] = b'h';

        let commit = slice.commit();
        file_1.commit(commit).unwrap();

        std::io::Seek::rewind(&mut file_1).unwrap();

        let mut file_1_content = String::new();
        std::io::Read::read_to_string(&mut file_1, &mut file_1_content).unwrap();

        let file_2_content = String::from_utf8(fs::read("./tests/dev/device_file_2.txt").unwrap()).unwrap();

        assert_eq!(file_1_content, file_2_content);
    }

    #[allow(clippy::struct_field_names)]
    #[test]
    fn device_generic_read_from_bytes() {
        const OFFSET: usize = 0xA0;

        #[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
        #[deku(endian = "little")]
        struct Test {
            nb_1: u16,
            nb_2: u8,
            nb_3: usize,
            nb_4: u128,
        }

        let test = Test {
            nb_1: 0xabcd,
            nb_2: 0x99,
            nb_3: 0x1234,
            nb_4: 0x1234_5678_90ab_cdef,
        };
        let test_bytes = test.to_bytes().unwrap();

        let mut device = Wrapper::new(vec![0_u8; 1024]);
        let mut slice = device.slice(Address::from(OFFSET)..Address::from(OFFSET + test_bytes.len())).unwrap();
        let buffer = slice.get_mut(..).unwrap();
        buffer.clone_from_slice(&test_bytes);

        let commit = slice.commit();
        device.commit(commit).unwrap();

        let read_test = device.read_from_bytes::<Test>(Address::from(OFFSET), 32).unwrap();
        assert_eq!(test, read_test);
    }

    #[allow(clippy::struct_field_names)]
    #[test]
    fn device_generic_write_to_bytes() {
        const OFFSET: u64 = 123;

        #[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
        struct Test {
            nb_1: u16,
            nb_2: u8,
            nb_3: usize,
            nb_4: u128,
        }

        let test = Test {
            nb_1: 0xabcd,
            nb_2: 0x99,
            nb_3: 0x1234,
            nb_4: 0x1234_5678_90ab_cdef,
        };
        let test_bytes = test.to_bytes().unwrap();

        let mut device = Wrapper::new(vec![0_u8; 1024]);
        device.write_to_bytes(Address::from(OFFSET), test).unwrap();

        let slice = device
            .slice(Address::from(OFFSET)..Address::from(OFFSET + test_bytes.len() as u64))
            .unwrap();

        assert_eq!(test_bytes, slice.as_ref());
    }

    #[test]
    fn dummy_device() {
        struct Foo {}

        impl Read for Foo {
            fn read(&mut self, buf: &mut [u8]) -> deku::no_std_io::Result<usize> {
                buf.fill(1);
                Ok(buf.len())
            }
        }

        impl Write for Foo {
            fn write(&mut self, buf: &[u8]) -> deku::no_std_io::Result<usize> {
                Ok(buf.len())
            }

            fn flush(&mut self) -> deku::no_std_io::Result<()> {
                Ok(())
            }
        }

        impl Seek for Foo {
            fn seek(&mut self, _pos: SeekFrom) -> deku::no_std_io::Result<u64> {
                Ok(0)
            }
        }

        #[derive(Debug, PartialEq, Eq, DekuRead)]
        struct Bar {
            a: u8,
            b: u32,
        }

        let mut device = Foo {};
        assert_eq!(device.read_from_bytes::<Bar>(Address::new(0), 5).unwrap(), Bar {
            a: 0x1,
            b: 0x0101_0101
        });
    }

    mod generated {
        use crate::tests::generate_fs_test;

        generate_fs_test!(device_file_write, "./tests/dev/device_file_1.txt");
    }
}
