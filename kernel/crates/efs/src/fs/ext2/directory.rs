//! Interface with ext2's directories.
//!
//! See the [OSdev wiki](https://wiki.osdev.org/Ext2#Directories) and the [*The Second Extended Filesystem* book](https://www.nongnu.org/ext2-doc/ext2.html#directory) for more information.

use alloc::ffi::CString;
use alloc::vec::Vec;
use core::fmt::Debug;

use deku::{DekuError, DekuRead, DekuWrite};

use super::Ext2;
use super::error::Ext2Error;
use crate::arch::u32_to_usize;
use crate::dev::Device;
use crate::dev::address::Address;
use crate::error::Error;
use crate::fs::file::Type;

/// A directory entry.
#[derive(Debug, Clone, DekuRead, DekuWrite)]
#[deku(endian = "little")]
pub struct Entry {
    /// Inode index.
    pub inode: u32,

    /// Total size of this entry (including all headers and the name).
    pub rec_len: u16,

    /// Name Length least-significant 8 bits.
    pub name_len: u8,

    /// Type indicator (only if the feature bit for "directory entries have file type byte" is set, else this is the
    /// most-significant 8 bits of the Name Length).
    pub file_type: u8,

    /// Name of the directory entry.
    #[deku(
        bytes_read = "*name_len",
        map = "|bytes: Vec<u8>| -> Result<_, DekuError> { CString::new(bytes).map_err(|_| DekuError::Io(deku::no_std_io::ErrorKind::InvalidData)) }"
    )]
    pub name: CString,
}

/// File type indicated in a directory entry.
///
/// See the [*The Second Extended Filesystem* book](https://www.nongnu.org/ext2-doc/ext2.html#ifdir-file-type) for more information.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileType {
    /// Unknown file type.
    Unknown = 0,

    /// Regular file.
    RegFile = 1,

    /// Directory.
    Dir = 2,

    /// Character device.
    ChrDev = 3,

    /// Block device.
    BlkDev = 4,

    /// FIFO.
    Fifo = 5,

    /// UNIX socket.
    Sock = 6,

    /// Symbolic link
    Symlink = 7,
}

impl From<u8> for FileType {
    fn from(value: u8) -> Self {
        match value {
            1 => Self::RegFile,
            2 => Self::Dir,
            3 => Self::ChrDev,
            4 => Self::BlkDev,
            5 => Self::Fifo,
            6 => Self::Sock,
            7 => Self::Symlink,
            _ => Self::Unknown,
        }
    }
}

impl From<FileType> for u8 {
    fn from(value: FileType) -> Self {
        value as Self
    }
}

impl From<Type> for FileType {
    fn from(value: Type) -> Self {
        match value {
            Type::Regular => Self::RegFile,
            Type::Directory => Self::Dir,
            Type::SymbolicLink => Self::Symlink,
            Type::Fifo => Self::Fifo,
            Type::CharacterDevice => Self::ChrDev,
            Type::BlockDevice => Self::BlkDev,
            Type::Socket => Self::Sock,
        }
    }
}

impl TryFrom<FileType> for Type {
    type Error = Ext2Error;

    fn try_from(value: FileType) -> Result<Self, Self::Error> {
        match value {
            FileType::Unknown => Err(Ext2Error::UnknownEntryFileType),
            FileType::RegFile => Ok(Self::Regular),
            FileType::Dir => Ok(Self::Directory),
            FileType::ChrDev => Ok(Self::CharacterDevice),
            FileType::BlkDev => Ok(Self::BlockDevice),
            FileType::Fifo => Ok(Self::Fifo),
            FileType::Sock => Ok(Self::Socket),
            FileType::Symlink => Ok(Self::SymbolicLink),
        }
    }
}

impl Entry {
    /// Returns the directory entry starting at the given address.
    ///
    /// # Errors
    ///
    /// Returns an [`Ext2Error::BadString`] if the name of the entry is not a valid C-string (non-null terminated).
    ///
    /// Returns an [`Error::IO`] if the device cannot be read.
    pub fn parse<Dev: Device>(fs: &Ext2<Dev>, starting_addr: Address) -> Result<Self, Error<Ext2Error>> {
        let mut device = fs.device.lock();
        device
            .read_from_bytes(starting_addr, u32_to_usize(fs.superblock().block_size()))
            .map_err(Into::into)
    }

    /// Returns the minimal size in bytes that this entry could take (with no consideration for `rec_len`).
    ///
    /// # Panics
    ///
    /// Cannot panic on an entry obtained with [`parse`](struct.Entry.html#method.parse): can only panic by creating by
    /// hand a ill-formed directory entry (whose length is greater than [`u16::MAX`]).
    #[must_use]
    pub fn minimal_size(&self) -> u16 {
        let minimal_size = u16::try_from(8 + self.name.to_bytes_with_nul().len()).expect("Ill-formed directory entry");
        minimal_size + (4 - ((minimal_size - 1) % 4 + 1))
    }

    /// Returns the free space contained in this entry.
    ///
    /// # Panics
    ///
    /// Cannot panic on an entry obtained with [`parse`](struct.Entry.html#method.parse): can only panic by creating by
    /// hand a ill-formed directory entry (whose length is greater than [`u16::MAX`]).
    #[must_use]
    pub fn free_space(&self) -> u16 {
        self.rec_len - u16::try_from(8 + self.name.to_bytes_with_nul().len()).expect("Ill-formed directory entry")
    }

    /// Returns this entry as bytes.
    #[must_use]
    pub fn as_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();

        bytes.append(&mut self.inode.to_le_bytes().to_vec());
        bytes.append(&mut self.rec_len.to_le_bytes().to_vec());
        bytes.push(self.name_len);
        bytes.push(self.file_type);
        bytes.append(&mut self.name.to_bytes_with_nul().to_vec());

        bytes
    }
}
