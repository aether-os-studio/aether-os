//! Interface for the structures in the SFS's Index Area.
//!
//! See the [OSDev wiki](https://wiki.osdev.org/SFS#Index_Area) and the [version 1.0 specification](https://web.archive.org/web/20170315134201/https://www.d-rift.nl/combuster/vdisk/sfs.html#Index_Area).

use alloc::vec::Vec;

use deku::{DekuRead, DekuWrite};

use super::SfsFs;
use super::error::SfsError;
use super::name_string::NameString;
use super::super_block::SuperBlock;
use super::time_stamp::TimeStamp;
use crate::celled::Celled;
use crate::dev::Device;
use crate::dev::address::Address;
use crate::error::Error;
use crate::fs::error::FsError;

/// Size in bytes of an entry in the Index Area.
pub const ENTRY_SIZE: u64 = 64;

/// Types of entries of the Index Area.
///
/// Any entry with a type not listed below can be consider and replaced by an [`UnusedEntry`], indicated by the
/// [`Unused`](EntryType::Unused) variant.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntryType {
    /// Indicates the [`VolumeIdentifierEntry`].
    VolumeIdentifier = 0x01,

    /// Indicates the [`StartingMarkerEntry`].
    StartingMarker = 0x02,

    /// Indicates a [`UnusedEntry`].
    Unused = 0x10,

    /// Indicates a [`DirectoryEntry`].
    Directory = 0x11,

    /// Indicates a [`FileEntry`].
    File = 0x12,

    /// Indicates a [`UnusableEntry`].
    Unusable = 0x18,

    /// Indicates a [`DeletedDirectoryEntry`].
    DeletedDirectory = 0x19,

    /// Indicates a [`DeletedFileEntry`].
    DeletedFile = 0x1A,

    /// Indicates a [`ContinuationEntry`].
    ///
    /// Valid values are between `0x20` and `0xFF` inclusive.
    Continuation(u8),
}

impl From<u8> for EntryType {
    fn from(value: u8) -> Self {
        match value {
            0x01 => Self::VolumeIdentifier,
            0x02 => Self::StartingMarker,
            0x11 => Self::Directory,
            0x12 => Self::File,
            0x18 => Self::Unusable,
            0x19 => Self::DeletedDirectory,
            0x1A => Self::DeletedFile,
            i @ 0x20..=0xFF => Self::Continuation(i),
            0x10 | _ => Self::Unused,
        }
    }
}

impl From<EntryType> for u8 {
    fn from(value: EntryType) -> Self {
        match value {
            EntryType::VolumeIdentifier => 0x01,
            EntryType::StartingMarker => 0x02,
            EntryType::Unused => 0x10,
            EntryType::Directory => 0x11,
            EntryType::File => 0x12,
            EntryType::Unusable => 0x18,
            EntryType::DeletedDirectory => 0x19,
            EntryType::DeletedFile => 0x1A,
            EntryType::Continuation(i) => i,
        }
    }
}

/// Utility trait to manipulate the different kinds of entry.
pub(super) trait Entry: Sized {
    /// Checks the validity of the fields of the entry.
    ///
    /// # Errors
    ///
    /// Returns an [`SfsError`] if some field is ill-formed.
    fn validity_check(&self, super_block: &SuperBlock) -> Result<(), Error<SfsError>>;

    /// Parses sequence of bytes that is exactly an entry of the Index Area and ensures that every field is well-formed.
    ///
    /// # Errors
    ///
    /// Returns an [`SfsError`] if some field are ill-formed.
    fn parse(bytes: [u8; 64], super_block: &SuperBlock) -> Result<Self, Error<SfsError>>;
}

/// Volume Identifier Entry in the Index Area.
///
/// It is located at the first entry of the Index Area, which corresponds to the end of the device as the Index Area is
/// located as the end of the device.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
pub struct VolumeIdentifierEntry {
    /// Entry type.
    ///
    /// Must equal to `0x01`.
    pub entry_type: u8,

    /// Unused/reserved.
    ///
    /// Must be equal to `0x000000`.
    pub reserved: [u8; 3],

    /// Time stamp of the first formatting of the device.
    ///
    /// It is stored in the same format as all time stamps used by SFS (see [module
    /// documentation](../index.html#time-stamps)).
    pub format_time: i64,

    /// Volume name in UTF-8, including the `\0` final character.
    ///
    /// It follows the same format as all name fields used by SFS (see [module
    /// documentation](../index.html#name-strings)).
    pub volume_name: [u8; 52],
}

impl VolumeIdentifierEntry {
    /// Parses the [`volume_name` field](struct.VolumeIdentifierEntry.html#structfield.volume_name) into a
    /// [`NameString`].
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`NameString::new_from_start`].
    pub fn parse_volume_name(&self) -> Result<NameString, Error<SfsError>> {
        NameString::new_from_start(&self.volume_name)
    }

    /// Parses the [`format_time` field](struct.VolumeIdentifierEntry.html#structfield.fomrat_time) into a
    /// [`TimeStamp`].
    #[must_use]
    pub fn parse_format_time(&self) -> TimeStamp {
        TimeStamp::from(self.format_time)
    }
}

impl Entry for VolumeIdentifierEntry {
    fn validity_check(&self, _super_block: &SuperBlock) -> Result<(), Error<SfsError>> {
        if EntryType::VolumeIdentifier != self.entry_type.into() {
            return Err(Error::Fs(FsError::Implementation(SfsError::WrongEntryType {
                expected: EntryType::VolumeIdentifier,
                given: self.entry_type.into(),
            })));
        }

        self.parse_volume_name()?;

        if self.reserved == [0, 0, 0] {
            Ok(())
        } else {
            Err(Error::Fs(FsError::Implementation(SfsError::BadVolumeIdentifierEntry(self.reserved))))
        }
    }

    fn parse(bytes: [u8; 64], super_block: &SuperBlock) -> Result<Self, Error<SfsError>> {
        let entry = Self {
            entry_type: bytes[0],
            reserved: core::array::from_fn(|i| bytes[i + 1]),
            format_time: i64::from_le_bytes(core::array::from_fn(|i| bytes[i + 4])),
            volume_name: core::array::from_fn(|i| bytes[i + 12]),
        };

        entry.validity_check(super_block).map(|()| entry)
    }
}

/// Starting Marker Entry in the Index Area.
///
/// It is used to find the beginning of the index area during file system recovery. There must always be one Starting
/// Marker Entry, and there must never be any index area entries closer to the start of the media than the Starting
/// Marker Entry.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
pub struct StartingMarkerEntry {
    /// Entry type.
    ///
    /// Must equal to `0x01`.
    pub entry_type: u8,

    /// Unused/reserved.
    pub reserved: [u8; 63],
}

impl Entry for StartingMarkerEntry {
    fn validity_check(&self, _super_block: &SuperBlock) -> Result<(), Error<SfsError>> {
        if EntryType::StartingMarker == self.entry_type.into() {
            Ok(())
        } else {
            Err(Error::Fs(FsError::Implementation(SfsError::WrongEntryType {
                expected: EntryType::StartingMarker,
                given: self.entry_type.into(),
            })))
        }
    }

    fn parse(bytes: [u8; 64], super_block: &SuperBlock) -> Result<Self, Error<SfsError>> {
        let entry = Self {
            entry_type: bytes[0],
            reserved: core::array::from_fn(|i| bytes[i + 1]),
        };

        entry.validity_check(super_block).map(|()| entry)
    }
}

/// Unused Entry in the Index Area.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
pub struct UnusedEntry {
    /// Entry type.
    ///
    /// Entry that are created as unused must equal to `0x10`, but all entries whose `entry_type` field is not included
    /// in any variant of [`EntryType`] are considered as unused.
    pub entry_type: u8,

    /// Unused/reserved.
    pub reserved: [u8; 63],
}

impl Entry for UnusedEntry {
    fn validity_check(&self, _super_block: &SuperBlock) -> Result<(), Error<SfsError>> {
        if EntryType::Unused == self.entry_type.into() {
            Ok(())
        } else {
            Err(Error::Fs(FsError::Implementation(SfsError::WrongEntryType {
                expected: EntryType::Unused,
                given: self.entry_type.into(),
            })))
        }
    }

    fn parse(bytes: [u8; 64], super_block: &SuperBlock) -> Result<Self, Error<SfsError>> {
        let entry = Self {
            entry_type: bytes[0],
            reserved: core::array::from_fn(|i| bytes[i + 1]),
        };

        entry.validity_check(super_block).map(|()| entry)
    }
}

/// Directory Entry representing a dictory of the filesystem.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
pub struct DirectoryEntry {
    /// Entry type.
    ///
    /// Must equal to `0x11`.
    pub entry_type: u8,

    /// Number of [`ContinuationEntry`] following this entry.
    pub continuation_nb: u8,

    /// Time stamp indicating when the directory was last modified.
    ///
    /// It is stored in the same format as all time stamps used by SFS (see [module
    /// documentation](../index.html#time-stamps)).
    pub last_modification_time: i64,

    /// Full path to the current directory.
    ///
    /// This directory name is the last component of the path split by `/`.
    ///
    /// The full string must end with a `\0` character and mustn't start with a `/` character..
    ///
    /// It follows the same format as all name fields used by SFS (see [module
    /// documentation](../index.html#name-strings)).
    ///
    /// Example: `foo/bar`
    pub path: [u8; 54],
}

impl DirectoryEntry {
    /// Parses the [`last_modification_time` field](struct.DirectoryEntry.html#structfield.last_modification_time) into
    /// a [`TimeStamp`].
    #[must_use]
    pub fn parse_last_modification_time(&self) -> TimeStamp {
        TimeStamp::from(self.last_modification_time)
    }

    /// Parses the [`path` field](struct.DirectoryEntry.html#structfield.path) into a [`NameString`].
    ///
    /// This function does **NOT** parse the full path of the directory represented by this entry: more precisely, this
    /// function does not take into account the possible continuation entries.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`NameString::new_from_start`].
    pub fn parse_path(&self) -> Result<NameString, Error<SfsError>> {
        NameString::new_from_start(&self.path)
    }
}

impl Entry for DirectoryEntry {
    fn validity_check(&self, _super_block: &SuperBlock) -> Result<(), Error<SfsError>> {
        if EntryType::Directory != self.entry_type.into() {
            return Err(Error::Fs(FsError::Implementation(SfsError::WrongEntryType {
                expected: EntryType::Directory,
                given: self.entry_type.into(),
            })));
        }

        self.parse_path().map(|_| ())
    }

    fn parse(bytes: [u8; 64], super_block: &SuperBlock) -> Result<Self, Error<SfsError>> {
        let entry = Self {
            entry_type: bytes[0],
            continuation_nb: bytes[1],
            last_modification_time: i64::from_le_bytes(core::array::from_fn(|i| bytes[i + 2])),
            path: core::array::from_fn(|i| bytes[i + 10]),
        };

        entry.validity_check(super_block).map(|()| entry)
    }
}

/// File Entry representing a regular file of the filesystem.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
pub struct FileEntry {
    /// Entry type.
    ///
    /// Must equal to `0x12`.
    pub entry_type: u8,

    /// Number of [`ContinuationEntry`] following this entry.
    pub continuation_nb: u8,

    /// Time stamp indicating when the file was last modified.
    ///
    /// It is stored in the same format as all time stamps used by SFS (see [module
    /// documentation](../index.html#time-stamps)).
    pub last_modification_time: i64,

    /// Starting block of the region in the Data Area used to store the file's content.
    pub data_starting_block: u64,

    /// Ending block of the region in the Data Area used to store the file's content.
    ///
    /// The ending block number must be higher than the starting block number, unless the entry does not use any blocks
    /// in the data area (a zero length file) in which case the starting and ending block numbers should both be zero.
    pub data_ending_block: u64,

    /// File length in bytes.
    ///
    /// This value must not be larger than the starting and ending block numbers indicate, but may be significantly
    /// less to reserve space for data to be appended to a file.
    pub length: u64,

    /// Full path to the current file.
    ///
    /// This file name is the last component of the path split by `/`.
    ///
    /// The full string must end with a `\0` character and mustn't start with a `/` character..
    ///
    /// It follows the same format as all name fields used by SFS (see [module
    /// documentation](../index.html#name-strings)).
    ///
    /// Example: `foo/bar/boo.txt`
    pub path: [u8; 30],
}

impl FileEntry {
    /// Parses the [`last_modification_time` field](struct.FileEntry.html#structfield.last_modification_time) into
    /// a [`TimeStamp`].
    #[must_use]
    pub fn parse_last_modification_time(&self) -> TimeStamp {
        TimeStamp::from(self.last_modification_time)
    }

    /// Parses the [`path` field](struct.FileEntry.html#structfield.path) into a [`NameString`].
    ///
    /// This function does **NOT** parse the full path of the directory represented by this entry: more precisely, this
    /// function does not take into account the possible continuation entries.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`NameString::new_from_start`].
    pub fn parse_path(&self) -> Result<NameString, Error<SfsError>> {
        NameString::new_from_start(&self.path)
    }

    /// Checks whether the data region indicated by this entry is valid or not.
    ///
    /// It checks that the region start is before the end, that both fit on the volume, and that the number of blocks is
    /// exactly the one needed.
    #[must_use]
    pub const fn is_data_region_valid(&self, super_block: &SuperBlock) -> bool {
        let necessary_blocks =
            if self.length == 0 { 0 } else { ((self.length - 1) / (super_block.bytes_per_block() as u64)) + 1 };
        self.data_starting_block < self.data_ending_block
            && self.data_ending_block - self.data_starting_block == necessary_blocks
            && ((super_block.is_block_in_data_area(self.data_starting_block)
                && super_block.is_block_in_data_area(self.data_ending_block))
                || ((self.data_starting_block == 0) && (self.data_ending_block == 0) && (self.length == 0)))
    }
}

impl Entry for FileEntry {
    fn validity_check(&self, super_block: &SuperBlock) -> Result<(), Error<SfsError>> {
        if EntryType::File != self.entry_type.into() {
            return Err(Error::Fs(FsError::Implementation(SfsError::WrongEntryType {
                expected: EntryType::File,
                given: self.entry_type.into(),
            })));
        }

        self.parse_path()?;
        if self.is_data_region_valid(super_block) {
            Ok(())
        } else {
            Err(Error::Fs(FsError::Implementation(SfsError::BadDataRegion {
                region_start: self.data_starting_block,
                region_end: self.data_ending_block,
                length: self.length,
            })))
        }
    }

    fn parse(bytes: [u8; 64], super_block: &SuperBlock) -> Result<Self, Error<SfsError>> {
        let entry = Self {
            entry_type: bytes[0],
            continuation_nb: bytes[1],
            last_modification_time: i64::from_le_bytes(core::array::from_fn(|i| bytes[i + 2])),
            data_starting_block: u64::from_le_bytes(core::array::from_fn(|i| bytes[i + 10])),
            data_ending_block: u64::from_le_bytes(core::array::from_fn(|i| bytes[i + 18])),
            length: u64::from_le_bytes(core::array::from_fn(|i| bytes[i + 26])),
            path: core::array::from_fn(|i| bytes[i + 34]),
        };

        entry.validity_check(super_block).map(|()| entry)
    }
}

/// Unusable Entry.
///
/// It is used to indicate regions of the Data Area that are unusable (because of bad sectors for example).
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
pub struct UnusableEntry {
    /// Entry type.
    ///
    /// Must equal to `0x18`.
    pub entry_type: u8,

    /// Unused/reserved.
    pub reserved_1: [u8; 9],

    /// Starting block of the region in the Data Area that is unusable.
    pub data_starting_block: u64,

    /// Ending block of the region in the Data Area that is unusable.
    pub data_ending_block: u64,

    /// Unused/reserved.
    pub reserved_2: [u8; 38],
}

impl UnusableEntry {
    /// Checks whether the data region indicated by this entry is valid or not.
    ///
    /// It checks that the region start is before the end, and that the number of blocks is exactly the one needed.
    #[must_use]
    pub const fn is_data_region_valid(&self, super_block: &SuperBlock) -> bool {
        self.data_starting_block <= self.data_ending_block
            && super_block.is_block_in_data_area(self.data_starting_block)
            && super_block.is_block_in_data_area(self.data_ending_block)
    }
}

impl Entry for UnusableEntry {
    fn validity_check(&self, super_block: &SuperBlock) -> Result<(), Error<SfsError>> {
        if EntryType::Unusable != self.entry_type.into() {
            Err(Error::Fs(FsError::Implementation(SfsError::WrongEntryType {
                expected: EntryType::Unusable,
                given: self.entry_type.into(),
            })))
        } else if !self.is_data_region_valid(super_block) {
            Err(Error::Fs(FsError::Implementation(SfsError::BadDataRegion {
                region_start: self.data_starting_block,
                region_end: self.data_ending_block,
                length: (1 + self.data_ending_block - self.data_starting_block)
                    * u64::from(super_block.bytes_per_block()),
            })))
        } else {
            Ok(())
        }
    }

    fn parse(bytes: [u8; 64], super_block: &SuperBlock) -> Result<Self, Error<SfsError>> {
        let entry = Self {
            entry_type: bytes[0],
            reserved_1: core::array::from_fn(|i| bytes[i + 1]),
            data_starting_block: u64::from_le_bytes(core::array::from_fn(|i| bytes[i + 10])),
            data_ending_block: u64::from_le_bytes(core::array::from_fn(|i| bytes[i + 18])),
            reserved_2: core::array::from_fn(|i| bytes[i + 26]),
        };

        entry.validity_check(super_block).map(|()| entry)
    }
}

/// Deleted Directory Entry representing a deleted dictory of the filesystem.
///
/// Such entries may be overwritten but they should keep the same attributes as the [`DirectoryEntry`] to be able to
/// restore one deleted directory by only changing the
/// [`entry_type`](struct.DeletedDirectoryEntry.html#structfield.entry_type) field.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
pub struct DeletedDirectoryEntry {
    /// Entry type.
    ///
    /// Must equal to `0x19`.
    pub entry_type: u8,

    /// Number of [`ContinuationEntry`] following this entry.
    pub continuation_nb: u8,

    /// Time stamp indicating when the directory was last modified.
    ///
    /// It is stored in the same format as all time stamps used by SFS (see [module
    /// documentation](../index.html#time-stamps)).
    pub last_modification_time: i64,

    /// Full path to the current directory.
    ///
    /// This directory name is the last component of the path split by `/`.
    ///
    /// The full string must end with a `\0` character and mustn't start with a `/` character..
    ///
    /// It follows the same format as all name fields used by SFS (see [module
    /// documentation](../index.html#name-strings)).
    ///
    /// Example: `foo/bar`
    pub path: [u8; 54],
}

impl DeletedDirectoryEntry {
    /// Parses the [`last_modification_time`
    /// field](struct.DeletedDirectoryEntry.html#structfield.last_modification_time) into a [`TimeStamp`].
    #[must_use]
    pub fn parse_last_modification_time(&self) -> TimeStamp {
        TimeStamp::from(self.last_modification_time)
    }

    /// Parses the [`path` field](struct.DeletedDirectoryEntry.html#structfield.path) into a [`NameString`].
    ///
    /// This function does **NOT** parse the full path of the directory represented by this entry: more precisely, this
    /// function does not take into account the possible continuation entries.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`NameString::new_from_start`].
    pub fn parse_path(&self) -> Result<NameString, Error<SfsError>> {
        NameString::new_from_start(&self.path)
    }
}

impl From<DirectoryEntry> for DeletedDirectoryEntry {
    fn from(value: DirectoryEntry) -> Self {
        Self {
            entry_type: value.entry_type,
            continuation_nb: value.continuation_nb,
            last_modification_time: value.last_modification_time,
            path: value.path,
        }
    }
}

impl From<DeletedDirectoryEntry> for DirectoryEntry {
    fn from(value: DeletedDirectoryEntry) -> Self {
        Self {
            entry_type: value.entry_type,
            continuation_nb: value.continuation_nb,
            last_modification_time: value.last_modification_time,
            path: value.path,
        }
    }
}

impl Entry for DeletedDirectoryEntry {
    fn validity_check(&self, _super_block: &SuperBlock) -> Result<(), Error<SfsError>> {
        if EntryType::DeletedDirectory != self.entry_type.into() {
            return Err(Error::Fs(FsError::Implementation(SfsError::WrongEntryType {
                expected: EntryType::DeletedDirectory,
                given: self.entry_type.into(),
            })));
        }

        self.parse_path().map(|_| ())
    }

    fn parse(bytes: [u8; 64], super_block: &SuperBlock) -> Result<Self, Error<SfsError>> {
        let entry = Self {
            entry_type: bytes[0],
            continuation_nb: bytes[1],
            last_modification_time: i64::from_le_bytes(core::array::from_fn(|i| bytes[i + 2])),
            path: core::array::from_fn(|i| bytes[i + 10]),
        };

        entry.validity_check(super_block).map(|()| entry)
    }
}

/// Deleted File Entry representing a deleted regular file of the filesystem.
///
/// Such entries may be overwritten but they should keep the same attributes as the [`FileEntry`] to be able to
/// restore one deleted directory by only changing the
/// [`entry_type`](struct.DeletedFileEntry.html#structfield.entry_type) field.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
pub struct DeletedFileEntry {
    /// Entry type.
    ///
    /// Must equal to `0x1A`.
    pub entry_type: u8,

    /// Number of [`ContinuationEntry`] following this entry.
    pub continuation_nb: u8,

    /// Time stamp indicating when the file was last modified.
    ///
    /// It is stored in the same format as all time stamps used by SFS (see [module
    /// documentation](../index.html#time-stamps)).
    pub last_modification_time: i64,

    /// Starting block of the region in the Data Area used to store the file's content.
    pub data_starting_block: u64,

    /// Ending block of the region in the Data Area used to store the file's content.
    ///
    /// The ending block number must be higher than the starting block number, unless the entry does not use any blocks
    /// in the data area (a zero length file) in which case the starting and ending block numbers should both be zero.
    pub data_ending_block: u64,

    /// File length in bytes.
    ///
    /// This value must not be larger than the starting and ending block numbers indicate, but may be significantly
    /// less to reserve space for data to be appended to a file.
    pub length: u64,

    /// Full path to the current file.
    ///
    /// This file name is the last component of the path split by `/`.
    ///
    /// The full string must end with a `\0` character and mustn't start with a `/` character..
    ///
    /// It follows the same format as all name fields used by SFS (see [module
    /// documentation](../index.html#name-strings)).
    ///
    /// Example: `foo/bar/boo.txt`
    pub path: [u8; 30],
}

impl DeletedFileEntry {
    /// Parses the [`last_modification_time` field](struct.DeletedFileEntry.html#structfield.last_modification_time)
    /// into a [`TimeStamp`].
    #[must_use]
    pub fn parse_last_modification_time(&self) -> TimeStamp {
        TimeStamp::from(self.last_modification_time)
    }

    /// Parses the [`path` field](struct.DeletedFileEntry.html#structfield.path) into a [`NameString`].
    ///
    /// This function does **NOT** parse the full path of the directory represented by this entry: more precisely, this
    /// function does not take into account the possible continuation entries.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`NameString::new_from_start`].
    pub fn parse_path(&self) -> Result<NameString, Error<SfsError>> {
        NameString::new_from_start(&self.path)
    }

    /// Checks whether the data region indicated by this entry is valid or not.
    ///
    /// It checks that the region start is before the end, that both fit on the volume, and that the number of blocks is
    /// exactly the one needed.
    #[must_use]
    pub const fn is_data_region_valid(&self, super_block: &SuperBlock) -> bool {
        let necessary_blocks =
            if self.length == 0 { 0 } else { ((self.length - 1) / (super_block.bytes_per_block() as u64)) + 1 };
        self.data_starting_block < self.data_ending_block
            && self.data_ending_block - self.data_starting_block == necessary_blocks
            && ((super_block.is_block_in_data_area(self.data_starting_block)
                && super_block.is_block_in_data_area(self.data_ending_block))
                || ((self.data_starting_block == 0) && (self.data_ending_block == 0) && (self.length == 0)))
    }
}

impl From<FileEntry> for DeletedFileEntry {
    fn from(value: FileEntry) -> Self {
        Self {
            entry_type: value.entry_type,
            continuation_nb: value.continuation_nb,
            last_modification_time: value.last_modification_time,
            data_starting_block: value.data_starting_block,
            data_ending_block: value.data_ending_block,
            length: value.length,
            path: value.path,
        }
    }
}

impl From<DeletedFileEntry> for FileEntry {
    fn from(value: DeletedFileEntry) -> Self {
        Self {
            entry_type: value.entry_type,
            continuation_nb: value.continuation_nb,
            last_modification_time: value.last_modification_time,
            data_starting_block: value.data_starting_block,
            data_ending_block: value.data_ending_block,
            length: value.length,
            path: value.path,
        }
    }
}

impl Entry for DeletedFileEntry {
    fn validity_check(&self, super_block: &SuperBlock) -> Result<(), Error<SfsError>> {
        if EntryType::DeletedFile != self.entry_type.into() {
            return Err(Error::Fs(FsError::Implementation(SfsError::WrongEntryType {
                expected: EntryType::DeletedFile,
                given: self.entry_type.into(),
            })));
        }

        self.parse_path()?;
        if self.is_data_region_valid(super_block) {
            Ok(())
        } else {
            Err(Error::Fs(FsError::Implementation(SfsError::BadDataRegion {
                region_start: self.data_starting_block,
                region_end: self.data_ending_block,
                length: self.length,
            })))
        }
    }

    fn parse(bytes: [u8; 64], super_block: &SuperBlock) -> Result<Self, Error<SfsError>> {
        let entry = Self {
            entry_type: bytes[0],
            continuation_nb: bytes[1],
            last_modification_time: i64::from_le_bytes(core::array::from_fn(|i| bytes[i + 2])),
            data_starting_block: u64::from_le_bytes(core::array::from_fn(|i| bytes[i + 10])),
            data_ending_block: u64::from_le_bytes(core::array::from_fn(|i| bytes[i + 18])),
            length: u64::from_le_bytes(core::array::from_fn(|i| bytes[i + 26])),
            path: core::array::from_fn(|i| bytes[i + 34]),
        };

        entry.validity_check(super_block).map(|()| entry)
    }
}

/// Continuation Entry in the Index Area.
///
/// They are used when the name field in a preceding Directory Entry, File Entry, Deleted Directory Entry or Deleted
/// File Entry is not long enough to contain the entire name.
///
/// Name string are directly concatenated and only one `<NUL>` character must be present, at the very end of the entry
/// name.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
pub struct ContinuationEntry {
    /// It follows the same format as all name fields used by SFS (see [module
    /// documentation](../index.html#name-strings)).
    entry_name: [u8; 64],
}

impl ContinuationEntry {
    /// Parses the [`path` field](struct.ContinuationEntry.html#structfield.path) into a [`NameString`].
    ///
    /// This function does **NOT** parse the full path of the directory represented by this entry: more precisely, this
    /// function does not take into account the possible continuation entries.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`NameString::new_from_start`].
    pub fn parse_entry_name(&self) -> Result<NameString, Error<SfsError>> {
        NameString::new_from_start(&self.entry_name)
    }
}

impl Entry for ContinuationEntry {
    fn validity_check(&self, _super_block: &SuperBlock) -> Result<(), Error<SfsError>> {
        if EntryType::Continuation(self.entry_name[0]) != self.entry_name[0].into() {
            return Err(Error::Fs(FsError::Implementation(SfsError::WrongEntryType {
                expected: EntryType::DeletedFile,
                given: self.entry_name[0].into(),
            })));
        }

        self.parse_entry_name().map(|_| ())
    }

    fn parse(bytes: [u8; 64], super_block: &SuperBlock) -> Result<Self, Error<SfsError>> {
        let entry = Self { entry_name: bytes };

        entry.validity_check(super_block).map(|()| entry)
    }
}

/// Enumeration of possible entries in the Index Area.
#[derive(Debug, Clone, Copy)]
pub enum EntryTypeWithEntry {
    /// Indicates the end of the volume.
    VolumeIdentifier(VolumeIdentifierEntry),

    /// Indicates the start of the Index Area.
    StartingMarker(StartingMarkerEntry),

    /// Unused entry.
    Unused(UnusedEntry),

    /// The representation of a directory in SFS.
    Directory(DirectoryEntry),

    /// The representation of a regular file in SFS.
    File(FileEntry),

    /// Indicates an unusable area in the Data Area.
    Unusable(UnusableEntry),

    /// The representation of a deleted directory in SFS.
    DeletedDirectory(DeletedDirectoryEntry),

    /// The representation of a deleted regular file in SFS.
    DeletedFile(DeletedFileEntry),

    /// Used to extend a file name.
    Continuation(ContinuationEntry),
}

impl EntryTypeWithEntry {
    /// Parses an entry of the Index Area from its bytes and returns its corresponding variant.
    ///
    /// This function ensure that every field of the resulting entry is well-formed.
    ///
    /// # Errors
    ///
    /// Returns a [`SfsError`] depending on the entry type and the error encountered during the parsing.
    pub fn parse_bytes(bytes: [u8; 64], super_block: &SuperBlock) -> Result<Self, Error<SfsError>> {
        let enty_type = EntryType::from(bytes[0]);

        match enty_type {
            EntryType::VolumeIdentifier => {
                Ok(Self::VolumeIdentifier(VolumeIdentifierEntry::parse(bytes, super_block)?))
            },
            EntryType::StartingMarker => Ok(Self::StartingMarker(StartingMarkerEntry::parse(bytes, super_block)?)),
            EntryType::Unused => Ok(Self::Unused(UnusedEntry::parse(bytes, super_block)?)),
            EntryType::Directory => Ok(Self::Directory(DirectoryEntry::parse(bytes, super_block)?)),
            EntryType::File => Ok(Self::File(FileEntry::parse(bytes, super_block)?)),
            EntryType::Unusable => Ok(Self::Unusable(UnusableEntry::parse(bytes, super_block)?)),
            EntryType::DeletedDirectory => {
                Ok(Self::DeletedDirectory(DeletedDirectoryEntry::parse(bytes, super_block)?))
            },
            EntryType::DeletedFile => Ok(Self::DeletedFile(DeletedFileEntry::parse(bytes, super_block)?)),
            EntryType::Continuation(_) => Ok(Self::Continuation(ContinuationEntry::parse(bytes, super_block)?)),
        }
    }

    /// Parses an entry of the Index Area and returns the corresponding variant.
    ///
    /// This function ensure that every field of the resulting entry is well-formed.
    ///
    /// The entries are indexed in the order of the Index Area, i.e in the **reverse order** of the device.
    ///
    /// # Errors
    ///
    /// Returns a [`SfsError`] depending on the entry type and the error encountered during the parsing.
    ///
    /// Returns an [`Error::IO`] if the device cannot be read.
    ///
    /// # Safety
    ///
    /// Must ensure that an entry entry is located at `starting_addr`.
    ///
    /// Must also ensure the requirements of [`Device::slice`].
    unsafe fn parse_at<Dev: Device>(
        celled_device: &Celled<Dev>,
        super_block: &SuperBlock,
        starting_addr: Address,
    ) -> Result<Self, Error<SfsError>> {
        let mut device = celled_device.lock();

        let slice = device.slice(starting_addr..starting_addr + ENTRY_SIZE)?;
        let bytes: [u8; 64] = unsafe { (*slice).try_into().unwrap_unchecked() };
        Self::parse_bytes(bytes, super_block)
    }

    /// Returns the starting address of the entry at the given index in the Index Area.
    ///
    /// The entries are indexed in the order of the Index Area, i.e in the **reverse order** of the device.
    #[must_use]
    pub fn starting_addr(super_block: &SuperBlock, index: u64) -> Address {
        super_block.index_area_starting_addr() + super_block.index_size - index * ENTRY_SIZE
    }

    /// Parses an entry of the Index Area given by its index and returns the corresponding variant.
    ///
    /// This function ensure that every field of the resulting entry is well-formed.
    ///
    /// The entries are indexed in the order of the Index Area, i.e in the **reverse order** of the device.
    ///
    /// # Errors
    ///
    /// Returns a [`SfsError`] depending on the entry type and the error encountered during the parsing.
    ///
    /// Returns an [`Error::IO`] if the device cannot be read.
    pub fn parse<Dev: Device>(
        celled_device: &Celled<Dev>,
        super_block: &SuperBlock,
        index: u64,
    ) -> Result<Self, Error<SfsError>> {
        // SAFETY: the starting addr is calculated above
        unsafe { Self::parse_at(celled_device, super_block, Self::starting_addr(super_block, index)) }
    }

    /// Returns the [`EntryType`] corresponding to the current [`EntryTypeWithEntry`].
    #[must_use]
    pub const fn variant(&self) -> EntryType {
        match self {
            Self::VolumeIdentifier(_) => EntryType::VolumeIdentifier,
            Self::StartingMarker(_) => EntryType::StartingMarker,
            Self::Unused(_) => EntryType::Unused,
            Self::Directory(_) => EntryType::Directory,
            Self::File(_) => EntryType::File,
            Self::Unusable(_) => EntryType::Unusable,
            Self::DeletedDirectory(_) => EntryType::DeletedDirectory,
            Self::DeletedFile(_) => EntryType::DeletedFile,
            Self::Continuation(entry) => EntryType::Continuation(entry.entry_name[0]),
        }
    }
}

impl From<EntryTypeWithEntry> for EntryType {
    fn from(value: EntryTypeWithEntry) -> Self {
        value.variant()
    }
}

/// Returns the full path of the given entry. [`ContinuationEntry`] linked to the given entry will also be parsed.
///
/// If the parsed entry does not contain a path, returns [`None`].
///
/// The entries are indexed in the order of the Index Area, i.e in the **reverse order** of the device.
///
/// # Errors
///
/// Returns a [`SfsError::InvalidNameString`] if the path is not a valid [`NameString`].
///
/// Returns a [`Error::IO`] if the device cannot be read.
pub fn parse_full_path<Dev: Device>(
    celled_device: &Celled<Dev>,
    super_block: &SuperBlock,
    entry_number: u64,
) -> Result<Option<NameString>, Error<SfsError>> {
    let entry = EntryTypeWithEntry::parse(celled_device, super_block, entry_number)?;

    let (mut name, continuation_nb) = match entry {
        EntryTypeWithEntry::Directory(directory_entry) => {
            (directory_entry.parse_path()?, u64::from(directory_entry.continuation_nb))
        },
        EntryTypeWithEntry::File(file_entry) => (file_entry.parse_path()?, u64::from(file_entry.continuation_nb)),
        EntryTypeWithEntry::DeletedDirectory(deleted_directory_entry) => {
            (deleted_directory_entry.parse_path()?, u64::from(deleted_directory_entry.continuation_nb))
        },
        EntryTypeWithEntry::DeletedFile(deleted_file_entry) => {
            (deleted_file_entry.parse_path()?, u64::from(deleted_file_entry.continuation_nb))
        },
        EntryTypeWithEntry::Continuation(continuation_entry) => (continuation_entry.parse_entry_name()?, 0),
        _ => return Ok(None),
    };

    for idx in 1..=continuation_nb {
        let entry = EntryTypeWithEntry::parse(celled_device, super_block, entry_number + idx)?;
        let EntryTypeWithEntry::Continuation(continuation_entry) = entry else {
            return Err(Error::Fs(FsError::Implementation(SfsError::WrongEntryType {
                expected: EntryType::Continuation(0x20),
                given: entry.into(),
            })));
        };

        name.join(&continuation_entry.parse_entry_name()?);
    }

    Ok(Some(name))
}

/// Returns the list of entries of the Index Area until the first one that satisfies the given predicate (included). The
/// predicate takes as arguments an entry, its index in the Index Area and the device.
///
/// The entries are parsed in the order of the Index Area, i.e in the **reverse order** of the device.
///
/// # Errors
///
/// Returns a [`Error::IO`] if the device could not be read.
pub fn parse_entries_until<Dev: Device, F: Fn(EntryTypeWithEntry, u64) -> Result<bool, Error<SfsError>>>(
    filesystem: &SfsFs<Dev>,
    predicate: F,
) -> Result<Vec<EntryTypeWithEntry>, Error<SfsError>> {
    let fs = filesystem.lock();
    let super_block = fs.super_block();

    let mut entries = Vec::new();

    let starting_addr = super_block.index_area_starting_addr();
    let ending_byte = super_block.filesystem_size();
    let nb_entries = (ending_byte - starting_addr.index()) / 64;

    for idx in 1..=nb_entries {
        let entry = EntryTypeWithEntry::parse(&fs.device, super_block, idx)?;
        entries.push(entry);

        if predicate(entry, idx)? {
            break;
        }
    }

    Ok(entries)
}

/// Returns the first entry of the Index Area satisfiying the given predicate, if it exists, and the corresponding index
/// this entry.
///
/// The predicate takes as arguments an entry, its index in the Index Area and the device.
///
/// Returns [`None`] otherwise. The predicate takes as arguments an entry and its index in the Index Area.
///
/// The entries are parsed in the order of the Index Area, i.e in the **reverse order** of the device. The index is also
/// given following this order.
///
/// # Errors
///
/// Returns a [`Error::IO`] if the device could not be read.
pub fn find_entry<Dev: Device, F: Fn(EntryTypeWithEntry, u64, &Celled<Dev>) -> Result<bool, Error<SfsError>>>(
    filesystem: &SfsFs<Dev>,
    predicate: F,
) -> Result<Option<(EntryTypeWithEntry, u64)>, Error<SfsError>> {
    // This function does not use `parse_entries_until` to avoid unecessary allocation.
    let fs = filesystem.lock();
    let super_block = fs.super_block();

    let starting_addr = super_block.index_area_starting_addr();
    let ending_byte = super_block.filesystem_size();
    let nb_entries = (ending_byte - starting_addr.index()) / 64;

    for idx in 1..=nb_entries {
        let entry = EntryTypeWithEntry::parse(&fs.device, super_block, idx)?;
        if predicate(entry, idx, &fs.device)? {
            return Ok(Some((entry, idx + 1)));
        }
    }

    Ok(None)
}

/// Returns the full list of entries of the Index Area satisfiying the given predicate that takes as argument an entry,
/// its index in the Index Area and the device.
///
/// The entries are parsed in the order of the Index Area, i.e in the **reverse order** of the device.
///
/// # Errors
///
/// Returns a [`Error::IO`] if the device could not be read.
pub fn find_all_entries<Dev: Device, F: Fn(EntryTypeWithEntry, u64, &Celled<Dev>) -> Result<bool, Error<SfsError>>>(
    filesystem: &SfsFs<Dev>,
    predicate: F,
) -> Result<Vec<(EntryTypeWithEntry, u64)>, Error<SfsError>> {
    let fs = filesystem.lock();
    let super_block = fs.super_block();

    let mut entries = Vec::new();

    let starting_addr = super_block.index_area_starting_addr();
    let ending_byte = super_block.filesystem_size();
    let nb_entries = (ending_byte - starting_addr.index()) / 64;

    for idx in 1..=nb_entries {
        let entry = EntryTypeWithEntry::parse(&fs.device, super_block, idx)?;

        if predicate(entry, idx, &fs.device)? {
            entries.push((entry, idx));
        }
    }

    Ok(entries)
}

#[cfg(test)]
mod test {
    use core::str::FromStr;

    use spin::Lazy;

    use super::{Entry, UnusableEntry, VolumeIdentifierEntry};
    use crate::fs::sfs::index_area::{
        ContinuationEntry, DeletedDirectoryEntry, DeletedFileEntry, DirectoryEntry, EntryType, EntryTypeWithEntry,
        FileEntry, StartingMarkerEntry, UnusedEntry,
    };
    use crate::fs::sfs::name_string::NameString;
    use crate::fs::sfs::super_block::SuperBlock;
    use crate::fs::sfs::time_stamp::TimeStamp;

    static TEST_SUPER_BLOCK: Lazy<SuperBlock> = Lazy::new(|| SuperBlock {
        time_stamp: *TimeStamp::now().unwrap(),
        data_size: 10,
        index_size: 10 * 64,
        magic: *b"SFS",
        version: 0x10,
        total_blocks: 20,
        rsvd_blocks: 3,
        block_size: 1,
        crc: 0xEC,
    });

    const TEST_VOLUME_IDENTIFIER_ENTRY: [u8; 64] = [
        0x01, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, b'f', b'o', b'o', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x19, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];

    const TEST_STARTING_MARKER_ENTRY: [u8; 64] = [
        0x02, 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 5, 1, 44, 51, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 123, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];

    const TEST_UNUSED_ENTRY: [u8; 64] = [
        0x10, 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 5, 1, 44, 51, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 123, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];

    const TEST_DIRECTORY_ENTRY: [u8; 64] = [
        0x11, 1, 5, 0, 0, 0, 0, 0, 0, 0, b'f', b'o', b'o', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];

    const TEST_FILE_ENTRY: [u8; 64] = [
        0x12, 1, 5, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 88, 2, 0, 0, 0, 0, 0, 0, b'f',
        b'o', b'o', b'/', b'b', b'a', b'r', b'.', b't', b'x', b't', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0,
    ];

    const TEST_UNUSABLE_ENTRY: [u8; 64] = [
        0x18, 1, 2, 3, 4, 5, 6, 7, 8, 9, 7, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];

    const TEST_DELETED_DIRECTORY_ENTRY: [u8; 64] = [
        0x19, 1, 5, 0, 0, 0, 0, 0, 0, 0, b'f', b'o', b'o', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];

    const TEST_DELETED_FILE_ENTRY: [u8; 64] = [
        0x1A, 1, 5, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 88, 2, 0, 0, 0, 0, 0, 0, b'f',
        b'o', b'o', b'/', b'b', b'a', b'r', b'.', b't', b'x', b't', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0,
    ];

    const TEST_CONTINUATION_ENTRY: [u8; 64] = [
        b'f', b'o', b'o', b'/', b'b', b'a', b'r', b'.', b't', b'x', b't', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0,
    ];

    #[test]
    fn parse_volume_identifier_entry() {
        let entry = VolumeIdentifierEntry::parse(TEST_VOLUME_IDENTIFIER_ENTRY, &TEST_SUPER_BLOCK)
            .expect("Could not parse the test volume identifier entry");
        assert_eq!(EntryType::from(entry.entry_type), EntryType::VolumeIdentifier);
        assert_eq!(entry.parse_format_time(), TimeStamp::from(5));
        assert_eq!(entry.parse_volume_name().unwrap(), NameString::from_str("foo").unwrap());
    }

    #[test]
    fn parse_starting_marker_entry() {
        let entry = StartingMarkerEntry::parse(TEST_STARTING_MARKER_ENTRY, &TEST_SUPER_BLOCK)
            .expect("Could not parse the test starting marker entry");
        assert_eq!(EntryType::from(entry.entry_type), EntryType::StartingMarker);
    }

    #[test]
    fn parse_unused_entry() {
        let entry =
            UnusedEntry::parse(TEST_UNUSED_ENTRY, &TEST_SUPER_BLOCK).expect("Could not parse the test unused entry");
        assert_eq!(EntryType::from(entry.entry_type), EntryType::Unused);
    }

    #[test]
    fn parse_directory_entry() {
        let entry = DirectoryEntry::parse(TEST_DIRECTORY_ENTRY, &TEST_SUPER_BLOCK)
            .expect("Could not parse the test directory entry");
        assert_eq!(EntryType::from(entry.entry_type), EntryType::Directory);
        assert_eq!(entry.parse_last_modification_time(), TimeStamp::from(5));
        assert_eq!(entry.parse_path().unwrap(), NameString::from_str("foo").unwrap());
    }

    #[test]
    fn parse_file_entry() {
        let entry = FileEntry::parse(TEST_FILE_ENTRY, &TEST_SUPER_BLOCK).expect("Could not parse the test file entry");
        assert_eq!(EntryType::from(entry.entry_type), EntryType::File);
        assert_eq!(entry.parse_last_modification_time(), TimeStamp::from(5));
        assert_eq!(entry.parse_path().unwrap(), NameString::from_str("foo/bar.txt").unwrap());

        let data_starting_block = entry.data_starting_block;
        let data_ending_block = entry.data_ending_block;
        let length = entry.length;
        assert_eq!(data_starting_block, 6);
        assert_eq!(data_ending_block, 9);
        assert_eq!(length, 600);
    }

    #[test]
    fn parse_unusable_entry() {
        let entry = UnusableEntry::parse(TEST_UNUSABLE_ENTRY, &TEST_SUPER_BLOCK)
            .expect("Could not parse the test unusable entry");
        assert_eq!(EntryType::from(entry.entry_type), EntryType::Unusable);

        let data_starting_block = entry.data_starting_block;
        let data_ending_block = entry.data_ending_block;
        assert_eq!(data_starting_block, 7);
        assert_eq!(data_ending_block, 7);
    }

    #[test]
    fn parse_deleted_directory_entry() {
        let entry = DeletedDirectoryEntry::parse(TEST_DELETED_DIRECTORY_ENTRY, &TEST_SUPER_BLOCK)
            .expect("Could not parse the test deleted directory entry");
        assert_eq!(EntryType::from(entry.entry_type), EntryType::DeletedDirectory);
        assert_eq!(entry.parse_last_modification_time(), TimeStamp::from(5));
        assert_eq!(entry.parse_path().unwrap(), NameString::from_str("foo").unwrap());
    }

    #[test]
    fn parse_deleted_file_entry() {
        let entry = DeletedFileEntry::parse(TEST_DELETED_FILE_ENTRY, &TEST_SUPER_BLOCK)
            .expect("Could not parse the test deleted file entry");
        assert_eq!(EntryType::from(entry.entry_type), EntryType::DeletedFile);
        assert_eq!(entry.parse_last_modification_time(), TimeStamp::from(5));
        assert_eq!(entry.parse_path().unwrap(), NameString::from_str("foo/bar.txt").unwrap());

        let data_starting_block = entry.data_starting_block;
        let data_ending_block = entry.data_ending_block;
        let length = entry.length;
        assert_eq!(data_starting_block, 6);
        assert_eq!(data_ending_block, 9);
        assert_eq!(length, 600);
    }

    #[test]
    fn parse_continuation_entry() {
        let entry = ContinuationEntry::parse(TEST_CONTINUATION_ENTRY, &TEST_SUPER_BLOCK)
            .expect("Could not parse the test continuation entry");
        assert_eq!(EntryType::from(entry.entry_name[0]), EntryType::Continuation(entry.entry_name[0]));
        assert_eq!(entry.parse_entry_name().unwrap(), NameString::from_str("foo/bar.txt").unwrap());
    }

    #[test]
    fn parse_entries() {
        assert_eq!(
            EntryTypeWithEntry::parse_bytes(TEST_VOLUME_IDENTIFIER_ENTRY, &TEST_SUPER_BLOCK)
                .unwrap()
                .variant(),
            EntryType::VolumeIdentifier
        );
        assert_eq!(
            EntryTypeWithEntry::parse_bytes(TEST_STARTING_MARKER_ENTRY, &TEST_SUPER_BLOCK)
                .unwrap()
                .variant(),
            EntryType::StartingMarker
        );
        assert_eq!(
            EntryTypeWithEntry::parse_bytes(TEST_UNUSED_ENTRY, &TEST_SUPER_BLOCK).unwrap().variant(),
            EntryType::Unused
        );
        assert_eq!(
            EntryTypeWithEntry::parse_bytes(TEST_DIRECTORY_ENTRY, &TEST_SUPER_BLOCK)
                .unwrap()
                .variant(),
            EntryType::Directory
        );
        assert_eq!(
            EntryTypeWithEntry::parse_bytes(TEST_FILE_ENTRY, &TEST_SUPER_BLOCK).unwrap().variant(),
            EntryType::File
        );

        assert_eq!(
            EntryTypeWithEntry::parse_bytes(TEST_UNUSABLE_ENTRY, &TEST_SUPER_BLOCK).unwrap().variant(),
            EntryType::Unusable
        );
        assert_eq!(
            EntryTypeWithEntry::parse_bytes(TEST_DELETED_DIRECTORY_ENTRY, &TEST_SUPER_BLOCK)
                .unwrap()
                .variant(),
            EntryType::DeletedDirectory
        );
        assert_eq!(
            EntryTypeWithEntry::parse_bytes(TEST_DELETED_FILE_ENTRY, &TEST_SUPER_BLOCK)
                .unwrap()
                .variant(),
            EntryType::DeletedFile
        );
        assert_eq!(
            EntryTypeWithEntry::parse_bytes(TEST_CONTINUATION_ENTRY, &TEST_SUPER_BLOCK)
                .unwrap()
                .variant(),
            EntryType::Continuation(TEST_CONTINUATION_ENTRY[0])
        );
    }
}
