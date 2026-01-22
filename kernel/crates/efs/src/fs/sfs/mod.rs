//! # sfs
//!
//! Implementation of the Simple File System (sfs).
//!
//! See [its official specification](https://web.archive.org/web/20170315134201/https://www.d-rift.nl/combuster/vdisk/sfs.html),
//! [FYSOS specification](https://www.fysnet.net/blog/files/sfs.pdf) for the version 1.10 and [its OSDev page](https://wiki.osdev.org/SFS).
//!
//! Two versions of SFS are available, with huge breaking changes between them: version 1.0 and version 1.10 (also
//! called version 1.A). Currently, only the version 1.0 is supported.
//!
//! ## Description
//!
//! The SFS structure is such like this:
//!
//! ```txt
//! +-----------------------------------------------------------+
//! |                                                           |
//! |                         Super-block                       |  Fixed-size
//! |                                                           |
//! +-----------------------------------------------------------+
//! |                                                           |
//! |                        Reserved Area                      |  Fixed-size
//! |                                                           |
//! +-----------------------------+-----------------------------+
//! |                                                           |  |
//! |                          Data Area                        |  |
//! |                                                           |  v
//! +-----------------------------+-----------------------------+
//! |                                                           |
//! |                          Free Area                        |
//! |                                                           |
//! +-----------------------------+-----------------------------+
//! |                                                           |  ^
//! |                         Index Area                        |  |
//! |                                                           |  |
//! +-----------------------------+-----------------------------+
//! ```
//!
//! ## SFS structures
//!
//! - The [`Device`] is splitted in contiguous blocks that have all the same size in bytes. This is **NOT** the block as
//!   in block device, here "block" always refers to ext2's blocks. They start at 0, so the `n`th block will start at
//!   the adress `n * block_size`.
//!
//! - The [`SuperBlock`] contains important metadata about the filesystem (size of blocks, size of areas, ...).
//!
//! - The Data Block Area is where all of the file data is stored. Each file starts at a specified block indicated in
//!   the Index Data Area and is sequentially stored toward the end of the volume, block by block. Each file starts on a
//!   block boundary.
//!
//! - The Index Area is where all of the file metadata is stored. This area holds a count of 64-byte entries holding
//!   various formats of data for the file system.
//!
//! - The Free Area is the blocks between the Data Block Area and the Index Data Area. These blocks are free for use to
//!   resize the Data Block Area, by adding blocks toward the end of the volume, or to resize the Index Data Area by
//!   adding blocks toward the start of the volume.
//!
//! ## Formats
//!
//! ### Name strings
//!
//! All name strings used as part of the Simple File System use [UTF-8 format](https://www.rfc-editor.org/rfc/rfc3629),
//! according to the Unicode Specification published by the [Unicode Consortium](https://home.unicode.org).
//!
//! Contrary to POSIX paths (implemented in the [`path`](crate::path) module), some non-`\0` characters are forbidden in
//! SFS names. The exhaustive list of forbidden characters is the following:
//!
//! - Characters whose code is strictly below `0x20`
//!
//! - Characters whose code is included between `0x80` and `0x9F` (inclusive).
//!
//! - The following special characters: `"` (double quote, `0x22`), `*` (asterix, `0x2A`), `:` (colon, `0x3A`), `<`
//!   (less than sign, `0x3C`), `>` (greater than sign, `0x3E`), `?` (question mark, `0x3F`), `\` (backward slash,
//!   `0x5C`), `<DEL>` (delete, `0x7F`) and `<NBSP>` (no-break space, `0xA0`).
//!
//! In particular, the `/` character **is allowed** and is expected to be present in volume names as a directory
//! separator only. Thus, it is not allowed within a volume label.
//!
//! Moreover, all name strings are C-like, meaning they must be `\0`-terminating.
//!
//! ### Time stamps
//!
//! All time stamps are signed 64 bit values that represent the number of 1/65536ths of a second since the beginning of
//! the 1st of January 1970. For example, the value `0x00000000003C0000` would represent one minute past midnight on the
//! 1st of January 1970, while the value `0x0000000000000001` would represent roughly 15259 ns past midnight on the 1st
//! of January 1970. All time stamps are in UTC (Universal Co-ordinated Time) so that problems with time zones and
//! daylight savings are avoided.

use derive_more::derive::{Deref, DerefMut};
use error::SfsError;
use file::Directory;
use index_area::{EntryTypeWithEntry, find_entry, parse_full_path};
use name_string::ROOT_NAME_STRING;
use super_block::SuperBlock;

use super::FilesystemRead;
use super::error::FsError;
use crate::celled::Celled;
use crate::dev::Device;
use crate::error::Error;

pub mod block;
pub mod error;
pub mod file;
pub mod index_area;
pub mod name_string;
pub mod super_block;
pub mod time_stamp;

/// Interface to manipulate devices containing an SFS filesystem.
#[derive(Debug, Clone)]
pub struct Sfs<Dev: Device> {
    /// Device number of the device containing the SFS filesystem.
    device_id: u32,

    /// Device containing the SFS filesystem.
    device: Celled<Dev>,

    /// Superblock of the filesystem.
    super_block: SuperBlock,
}

impl<Dev: Device> Sfs<Dev> {
    /// Creates a new [`Sfs`] object from the given device that should contain an SFS filesystem and a given device
    /// ID.
    ///
    /// # Errors
    ///
    /// Returns [`SfsError::BadMagic`] if the magic number found in the super-block is not equal to
    /// [`SFS_SIGNATURE`](super_block::SFS_SIGNATURE).
    ///
    ///  Returns [`SfsError::BadIndexAreaSize`] if the size of the Index Area in the super-block is not a multiple of
    /// 64.
    ///
    ///
    /// Returns an [`Error::IO`] if the device cannot be read.
    pub fn new(device: Dev, device_id: u32) -> Result<Self, Error<SfsError>> {
        let celled_device = Celled::new(device);
        Self::new_celled(celled_device, device_id)
    }

    /// Creates a new [`Sfs`] object from the given celled device that should contain a SFS filesystem and a given
    /// device ID.
    ///
    /// # Errors
    ///
    /// Returns [`SfsError::BadMagic`] if the magic number found in the super-block is not equal to
    /// [`SFS_SIGNATURE`](super_block::SFS_SIGNATURE).
    ///
    ///  Returns [`SfsError::BadIndexAreaSize`] if the size of the Index Area in the super-block is not a multiple of
    /// 64.
    ///
    ///
    /// Returns an [`Error::IO`] if the device cannot be read.
    pub fn new_celled(celled_device: Celled<Dev>, device_id: u32) -> Result<Self, Error<SfsError>> {
        let super_block = SuperBlock::parse(&celled_device)?;
        Ok(Self {
            device_id,
            device: celled_device,
            super_block,
        })
    }

    /// Returns the [`SuperBlock`] of this filesystem.
    #[must_use]
    pub const fn super_block(&self) -> &SuperBlock {
        &self.super_block
    }
}

/// Main interface to manipulate a SFS filesystem.
#[derive(Debug, Deref, DerefMut)]
pub struct SfsFs<Dev: Device>(Celled<Sfs<Dev>>);

impl<Dev: Device> Clone for SfsFs<Dev> {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<Dev: Device> SfsFs<Dev> {
    /// Creates a new [`SfsFs`] object from the given device that should contain a SFS filesystem, and from the given
    /// device ID.
    ///
    /// # Errors
    ///
    /// Returns [`SfsError::BadMagic`] if the magic number found in the super-block is not equal to
    /// [`SFS_SIGNATURE`](super_block::SFS_SIGNATURE).
    ///
    ///  Returns [`SfsError::BadIndexAreaSize`] if the size of the Index Area in the super-block is not a multiple of
    /// 64.
    ///
    ///
    /// Returns an [`Error::IO`] if the device cannot be read.
    pub fn new(device: Dev, device_id: u32) -> Result<Self, Error<SfsError>> {
        Ok(Self(Celled::new(Sfs::new(device, device_id)?)))
    }

    /// Creates a new [`SfsFs`] object from the given celled device that should contain a SFS filesystem, and from the
    /// given device ID.
    ///
    /// # Errors
    ///
    /// Returns [`SfsError::BadMagic`] if the magic number found in the super-block is not equal to
    /// [`SFS_SIGNATURE`](super_block::SFS_SIGNATURE).
    ///
    ///  Returns [`SfsError::BadIndexAreaSize`] if the size of the Index Area in the super-block is not a multiple of
    /// 64.
    ///
    ///
    /// Returns an [`Error::IO`] if the device cannot be read.
    pub fn new_celled(device: Celled<Dev>, device_id: u32) -> Result<Self, Error<SfsError>> {
        Ok(Self(Celled::new(Sfs::new_celled(device, device_id)?)))
    }

    /// Returns a reference to the inner [`Sfs`] object.
    #[must_use]
    pub const fn sfs_interface(&self) -> &Celled<Sfs<Dev>> {
        &self.0
    }
}

impl<Dev: Device> FilesystemRead<Directory<Dev>> for SfsFs<Dev> {
    fn root(
        &self,
    ) -> Result<Directory<Dev>, Error<<crate::fs::sfs::file::Directory<Dev> as crate::fs::file::Base>::FsError>> {
        let sfs = self.sfs_interface().lock();
        let super_block = *sfs.super_block();
        drop(sfs);
        let (entry, idx) = find_entry(self, |_entry, idx, device| {
            Ok(parse_full_path(device, &super_block, idx)?.is_some_and(|path| path == *ROOT_NAME_STRING))
        })?
        .ok_or(FsError::Implementation(SfsError::NoRoot))?;
        match entry {
            EntryTypeWithEntry::Directory(root) => Directory::new(self, idx, root),
            _ => Err(Error::Fs(FsError::Implementation(SfsError::WrongEntryType {
                expected: index_area::EntryType::Directory,
                given: entry.into(),
            }))),
        }
    }

    fn double_slash_root(
        &self,
    ) -> Result<Directory<Dev>, Error<<crate::fs::sfs::file::Directory<Dev> as crate::fs::file::Base>::FsError>> {
        self.root()
    }
}
