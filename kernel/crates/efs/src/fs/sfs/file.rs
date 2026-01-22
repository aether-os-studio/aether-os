//! Interface to manipulate UNIX file on a SFS filesystem.

use alloc::str::pattern::Pattern;
use alloc::string::ToString;
use alloc::vec::Vec;
use core::fmt::Debug;
use core::str::FromStr;

use deku::no_std_io::{Read, Seek, SeekFrom};

use super::SfsFs;
use super::block::Block;
use super::error::SfsError;
use super::index_area::{DirectoryEntry, Entry, EntryTypeWithEntry, FileEntry, find_all_entries, parse_full_path};
use super::time_stamp::TimeStamp;
use crate::arch::u32_to_usize;
use crate::dev::Device;
use crate::error::Error;
use crate::fs::error::FsError;
use crate::fs::file::{self, Base, Stat, TypeWithFile};
use crate::fs::permissions::Permissions;
use crate::fs::types::{Blkcnt, Blksize, Gid, Ino, Mode, Nlink, Off, Uid};
use crate::path::{Path, UnixStr};

/// Implementation of a regular file.
pub struct Regular<Dev: Device> {
    /// SFS object associated with the device containing this file.
    filesystem: SfsFs<Dev>,

    /// Entry number (in the order of the Index Area) corresponding to this file.
    entry_number: u64,

    /// Entry of the Index Area corresponding to this file.
    entry: FileEntry,

    /// Read/Write offset in bytes (can be manipulated with [`Seek`]).
    io_offset: u64,
}

impl<Dev: Device> Debug for Regular<Dev> {
    fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        fmt.debug_struct("File")
            .field("entry_number", &self.entry_number)
            .field("entry", &self.entry)
            .finish_non_exhaustive()
    }
}

impl<Dev: Device> Clone for Regular<Dev> {
    fn clone(&self) -> Self {
        Self {
            filesystem: self.filesystem.clone(),
            entry_number: self.entry_number,
            entry: self.entry,
            io_offset: u64::default(),
        }
    }
}

impl<Dev: Device> Regular<Dev> {
    /// Returns a new SFS [`Regular`] from a [`SfsFs`] instance and the entry number of this file.
    ///
    /// # Errors
    ///
    /// Returns a [`SfsError`] if the given entry is ill-formed.
    pub fn new(filesystem: &SfsFs<Dev>, entry_number: u64, entry: FileEntry) -> Result<Self, Error<SfsError>> {
        let fs = filesystem.lock();
        entry.validity_check(fs.super_block())?;
        Ok(Self {
            filesystem: filesystem.clone(),
            entry_number,
            entry,
            io_offset: u64::default(),
        })
    }
}

impl<Dev: Device> Base for Regular<Dev> {
    type FsError = SfsError;
}

impl<Dev: Device> file::FileRead for Regular<Dev> {
    fn stat(&self) -> file::Stat {
        let fs = self.filesystem.lock();
        let super_block = fs.super_block();
        let time = TimeStamp::from(super_block.time_stamp).into();
        Stat {
            dev: crate::fs::types::Dev(fs.device_id),
            ino: Ino(self.entry_number),
            mode: Mode::from(Permissions::from_bits_truncate(0o000_777)),
            nlink: Nlink(1),
            uid: Uid::default(),
            gid: Gid::default(),
            rdev: crate::fs::types::Dev::default(),
            size: Off(self.entry.length.try_into().unwrap_or(-1)),
            atim: time,
            mtim: time,
            ctim: time,
            blksize: Blksize(u32_to_usize(super_block.bytes_per_block()).cast_signed()),
            blkcnt: Blkcnt(super_block.total_blocks.cast_signed()),
        }
    }

    fn get_type(&self) -> file::Type {
        file::Type::Regular
    }
}

impl<Dev: Device> Read for Regular<Dev> {
    fn read(&mut self, buf: &mut [u8]) -> deku::no_std_io::Result<usize> {
        let block_size = u32_to_usize(self.filesystem.lock().super_block().bytes_per_block());

        let file_size = self.entry.length;
        let buf_size = buf.len();
        // If file_size does not fit on a usize, then it must be higher than `buf_size`.
        let bytes_to_read = buf_size.min(TryInto::<usize>::try_into(file_size).unwrap_or(buf_size));
        let mut read_bytes = 0;
        let blocks_to_read = self.entry.data_starting_block..self.entry.data_ending_block;

        for block_index in blocks_to_read {
            let mut block = Block::new(self.filesystem.clone(), block_index);
            let Some(bytes) = buf.get_mut(read_bytes..(read_bytes + block_size).min(bytes_to_read)) else {
                return Err(deku::no_std_io::Error::new(
                    deku::no_std_io::ErrorKind::UnexpectedEof,
                    "EOF reached before block end",
                ));
            };
            block.read_exact(bytes)?;
            read_bytes += bytes.len();
        }

        // SAFETY: `read_bytes` is a `usize`, so this might cause a problem for files with a length of 9e18 B (~10^7
        // TB), which is very unlikely to happen
        self.seek(SeekFrom::Current(unsafe { i64::try_from(read_bytes).unwrap_unchecked() }))?;

        Ok(read_bytes)
    }
}

impl<Dev: Device> Seek for Regular<Dev> {
    fn seek(&mut self, pos: SeekFrom) -> deku::no_std_io::Result<u64> {
        // SAFETY: it is safe to assume that the file length is smaller than 2^63 bytes long
        let file_length = unsafe { i64::try_from(self.entry.length).unwrap_unchecked() };
        let previous_offset = self.io_offset;
        match pos {
            SeekFrom::Start(offset) => self.io_offset = offset,
            SeekFrom::End(back_offset) => {
                self.io_offset = TryInto::<u64>::try_into(file_length + back_offset).map_err(|_err| {
                    deku::no_std_io::Error::new(
                        deku::no_std_io::ErrorKind::InvalidInput,
                        "Invalid seek to a negative or overflowing position",
                    )
                })?;
            },
            SeekFrom::Current(add_offset) => {
                // SAFETY: it is safe to assume that the file has a length smaller than 2^63 bytes.
                let previous_offset_signed = unsafe { TryInto::<i64>::try_into(previous_offset).unwrap_unchecked() };
                self.io_offset = (previous_offset_signed + add_offset).try_into().map_err(|_err| {
                    deku::no_std_io::Error::new(
                        deku::no_std_io::ErrorKind::InvalidInput,
                        "Invalid seek to a negative or overflowing position",
                    )
                })?;
            },
        }

        if self.io_offset > self.entry.length {
            Err(deku::no_std_io::Error::new(
                deku::no_std_io::ErrorKind::InvalidInput,
                "Invalid seek to a negative or overflowing position",
            ))
        } else {
            Ok(previous_offset)
        }
    }
}

impl<Dev: Device> file::RegularRead for Regular<Dev> {}

/// Implementation of a regular file.
pub struct Directory<Dev: Device> {
    /// SFS object associated with the device containing this file.
    filesystem: SfsFs<Dev>,

    /// Entry number (in the order of the Index Area) corresponding to this file.
    entry_number: u64,

    /// Entry of the Index Area corresponding to this file.
    entry: DirectoryEntry,
}

impl<Dev: Device> Debug for Directory<Dev> {
    fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        fmt.debug_struct("Directory")
            .field("entry_number", &self.entry_number)
            .field("entry", &self.entry)
            .finish_non_exhaustive()
    }
}

impl<Dev: Device> Clone for Directory<Dev> {
    fn clone(&self) -> Self {
        Self {
            filesystem: self.filesystem.clone(),
            entry_number: self.entry_number,
            entry: self.entry,
        }
    }
}

impl<Dev: Device> Directory<Dev> {
    /// Returns a new SFS [`Directory`] from a [`SfsFs`] instance and the entry number of this file.
    ///
    /// # Errors
    ///
    /// Returns a [`SfsError`] if the given entry is ill-formed.
    pub fn new(filesystem: &SfsFs<Dev>, entry_number: u64, entry: DirectoryEntry) -> Result<Self, Error<SfsError>> {
        let fs = filesystem.lock();
        entry.validity_check(fs.super_block())?;
        Ok(Self {
            filesystem: filesystem.clone(),
            entry_number,
            entry,
        })
    }
}

impl<Dev: Device> Base for Directory<Dev> {
    type FsError = SfsError;
}

impl<Dev: Device> file::FileRead for Directory<Dev> {
    fn stat(&self) -> file::Stat {
        let fs = self.filesystem.lock();
        let super_block = fs.super_block();
        let time = TimeStamp::from(super_block.time_stamp).into();
        Stat {
            dev: crate::fs::types::Dev(fs.device_id),
            ino: Ino(self.entry_number),
            mode: Mode::from(Permissions::from_bits_truncate(0o000_777)),
            nlink: Nlink(1),
            uid: Uid::default(),
            gid: Gid::default(),
            rdev: crate::fs::types::Dev::default(),
            size: Off::default(),
            atim: time,
            mtim: time,
            ctim: time,
            blksize: Blksize(u32_to_usize(super_block.bytes_per_block()).cast_signed()),
            blkcnt: Blkcnt(super_block.total_blocks.cast_signed()),
        }
    }

    fn get_type(&self) -> file::Type {
        file::Type::Directory
    }
}

/// Dummy [`BlockDevice`](crate::fs::file::BlockDevice) structure used to implement the
/// [`Directory`](crate::fs::file::Directory) trait to [`Directory`] as this type of file does not exist in SFS.
pub struct BlockDevice(!);

/// Dummy [`CharacterDevice`](crate::fs::file::CharacterDevice) structure used to implement the
/// [`Directory`](crate::fs::file::Directory) trait to [`Directory`] as this type of file does not exist in SFS.
pub struct CharacterDevice(!);

/// Dummy [`Fifo`](crate::fs::file::Fifo) structure used to implement the
/// [`Directory`](crate::fs::file::Directory) trait to [`Directory`] as this type of file does not exist in SFS.
pub struct Fifo(!);

/// Dummy [`Socket`](crate::fs::file::Socket) structure used to implement the
/// [`Directory`](crate::fs::file::Directory) trait to [`Directory`] as this type of file does not exist in SFS.
pub struct Socket(!);

/// Dummy [`SymbolicLink`](crate::fs::file::SymbolicLink) structure used to implement the
/// [`Directory`](crate::fs::file::Directory) trait to [`Directory`] as this type of file does not exist in SFS.
pub struct SymbolicLink(!);

macro_rules! impl_file {
    ($id:ident) => {
        impl crate::fs::file::Base for $id {
            type FsError = SfsError;
        }

        impl crate::fs::file::FileRead for $id {
            fn stat(&self) -> Stat {
                unreachable!("This type is not instatiable")
            }

            fn get_type(&self) -> crate::fs::file::Type {
                crate::fs::file::Type::$id
            }
        }

        impl crate::fs::file::${concat($id, Read)} for $id {}
    };
}

impl_file!(BlockDevice);
impl_file!(CharacterDevice);
impl_file!(Fifo);
impl_file!(Socket);

impl crate::fs::file::Base for SymbolicLink {
    type FsError = SfsError;
}

impl crate::fs::file::FileRead for SymbolicLink {
    fn stat(&self) -> Stat {
        unreachable!("This type is not instatiable")
    }

    fn get_type(&self) -> file::Type {
        crate::fs::file::Type::SymbolicLink
    }
}

impl file::SymbolicLinkRead for SymbolicLink {
    fn get_pointed_file(&self) -> Result<&str, Error<Self::FsError>> {
        unreachable!("This type is not instatiable")
    }
}

impl<Dev: Device> file::DirectoryRead for Directory<Dev> {
    type BlockDevice = BlockDevice;
    type CharacterDevice = CharacterDevice;
    type Fifo = Fifo;
    type Regular = Regular<Dev>;
    type Socket = Socket;
    type SymbolicLink = SymbolicLink;

    fn entries(&self) -> Result<Vec<file::DirectoryEntry<'_, Self>>, Error<<Self as crate::fs::file::Base>::FsError>> {
        let fs = self.filesystem.lock();
        let super_block = fs.super_block();
        let dir_name = Into::<Path<'_>>::into(
            parse_full_path(&fs.device, super_block, self.entry_number)?
                .ok_or(FsError::Implementation(SfsError::NameStringExpected(self.entry_number)))?,
        )
        .to_string();

        let entries = find_all_entries(&self.filesystem, |entry, idx, device| match entry {
            EntryTypeWithEntry::Directory(_) | EntryTypeWithEntry::File(_) => {
                let name = Into::<Path<'_>>::into(
                    parse_full_path(device, super_block, idx)?
                        .ok_or(FsError::Implementation(SfsError::NameStringExpected(idx)))?,
                )
                .to_string();
                Ok(dir_name.is_prefix_of(&name))
            },
            _ => Ok(false),
        })?;

        let mut ret = Vec::new();

        for (entry, idx) in entries {
            let name_string = parse_full_path(&fs.device, super_block, idx)?
                .ok_or(FsError::Implementation(SfsError::NameStringExpected(idx)))?;
            let path = UnixStr::from(name_string);
            ret.push(file::DirectoryEntry {
                filename: UnixStr::from_str(path.split('/').next_back().ok_or(Error::Fs(FsError::Implementation(
                    SfsError::InvalidNameString(path.to_string().as_bytes().to_vec()),
                )))?)?,
                file: match entry {
                    EntryTypeWithEntry::File(file_entry) => {
                        TypeWithFile::Regular(Regular::new(&self.filesystem, idx, file_entry)?)
                    },
                    EntryTypeWithEntry::Directory(directory_entry) => {
                        TypeWithFile::Directory(Self::new(&self.filesystem, idx, directory_entry)?)
                    },
                    _ => return Err(Error::Fs(FsError::Implementation(SfsError::EntryTypeNotFile(entry.into())))),
                },
            });
        }

        Ok(ret)
    }
}
