//! Errors related to SFS manipulation.

use alloc::vec::Vec;

use derive_more::derive::Display;

use super::index_area::EntryType;
use super::super_block::SFS_SIGNATURE;
use crate::fs::types::Timespec;

/// Enumeration of possible errors encountered with SFS manipulation.
#[allow(clippy::module_name_repetitions)]
#[derive(Debug, PartialEq, Eq, Display)]
#[display("SFS Error: {_variant}")]
pub enum SfsError {
    /// A bad value for the block size in the [`SuperBlock`](super::super_block::SuperBlock) has been found.
    #[display("Bad Index Area Size: {_0} should not be 0")]
    BadBlockSize(u8),

    /// A bad checksum has been given, indicating hat an error occured at some point.
    #[display("Bad Checksum: {expected} expected while {given} given")]
    BadChecksum {
        /// Expected checksum.
        expected: u8,

        /// Given checksum.
        given: u8,
    },

    /// Tried to parse a data region whose start is after its end.
    #[display(
        "Bad Data Region: the data region whose start is {region_start}, whose end is {region_end} and whose length is {length} is not possible"
    )]
    BadDataRegion {
        /// Start of the data region.
        region_start: u64,

        /// End of the data region.
        region_end: u64,

        /// Length in bytes of the region.
        length: u64,
    },

    /// A bad value for the Index Area size in the [`SuperBlock`](super::super_block::SuperBlock) has been found.
    #[display("Bad Index Area Size: {_0} should be a multiple of 64")]
    BadIndexAreaSize(u64),

    /// A bad magic number has been found during the [`SuperBlock`](super::super_block::SuperBlock) parsing.
    #[display("Bad Magic: {_0:?} has been found while {SFS_SIGNATURE:?} was expected")]
    BadMagic([u8; 3]),

    /// A bad value for the reserved part of the [`VolumeIdentiferEntry`](super::index_area::VolumeIdentifierEntry) has
    /// been found during the parsing.
    #[display("Bad Volume Identifier Entry: {_0:?} has been found while [0, 0, 0] was expected")]
    BadVolumeIdentifierEntry([u8; 3]),

    /// A entry of given type has been found but is not convertable into a [`File`](crate::fs::file::File).
    #[display("Entry Type Not File: the given entry type is not convertable into a file")]
    EntryTypeNotFile(EntryType),

    /// Tried to parse a invalid name string.
    #[display("Invalid Name String: {_0:?} is not a valid name string")]
    InvalidNameString(Vec<u8>),

    /// The entry at the given index does not contain a name string.
    #[display("Name String Expected: the entry at index {_0} does not contain a name string")]
    NameStringExpected(u64),

    /// The filesystem does not contain a root directory.
    #[display("No Root: the filesystem does not contain a root directory")]
    NoRoot,

    /// Tried to convert a too big [`Timespec`] into a SFS [`TimeStamp`](super::time_stamp::TimeStamp).
    ///
    /// This error cannot happend before >100 000 years if you only deal with current time, so if you encounter it you
    /// are probably handling bad data.
    #[display("Time Stamp Out of Bounds: the timespec {_0:?} cannot be represented by a SFS time stamp")]
    TimeStampOutOfBounds(Timespec),

    /// Tried to assign a wrong type to an entry.
    #[display("Wrong Entry Type: {expected:?} entry type expected, {given:?} given")]
    WrongEntryType {
        /// Expected entry type.
        expected: EntryType,

        /// Given entry type.
        given: EntryType,
    },
}

impl core::error::Error for SfsError {}
