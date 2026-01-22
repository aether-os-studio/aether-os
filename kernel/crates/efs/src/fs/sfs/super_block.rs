//! Interface for the SFS's superblock.
//!
//! See the [OSDev wiki](https://wiki.osdev.org/SFS#Super-block) and the [version 1.10 release](https://www.fysnet.net/blog/files/sfs.pdf).

use deku::{DekuRead, DekuWrite};

use super::error::SfsError;
use crate::celled::Celled;
use crate::dev::Device;
use crate::dev::address::Address;
use crate::error::Error;
use crate::fs::error::FsError;

/// SFS signature, used to help confirm the presence of a SFS volume.
///
/// The value is `0x534653`, which is the ASCII code of "SFS".
pub const SFS_SIGNATURE: [u8; 3] = [0x53, 0x46, 0x53];

/// Starting byte of the super-block in a version 1.0 SFS storage device.
pub const SUPER_BLOCK_START_BYTE: usize = 0x0194;

/// Size of the super-block in bytes.
pub const SUPER_BLOCK_SIZE: usize = 42;

/// Enumeration of existing areas in SFS.
///
/// See [this paragraph](../index.html#description) for more information.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Area {
    /// Super Block Area.
    SuperBlock,

    /// Reserved Area.
    Reserved,

    /// Data Area.
    Data,

    /// Free Area.
    Free,

    /// Index Area.
    Index,
}

/// The SFS Super Block.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
pub struct SuperBlock {
    /// Last alteration time of super-block values ([`data_size`](struct.SuperBlock.html#structfield.data_size) or
    /// [`index_size`](struct.SuperBlock.html#structfield.index_size)).
    pub time_stamp: i64,

    /// Size of Data Area in blocks.
    pub data_size: u64,

    /// Size of Index Area in bytes.
    ///
    /// It must be a multiple of 64.
    pub index_size: u64,

    /// SFS magic number.
    pub magic: [u8; 3],

    /// SFS version.
    ///
    /// Two versions are available: 1.0 (`0x10`) and 1.10 (`0x1A`).
    pub version: u8,

    /// Total number of blocks in volume (including Reserved Area).
    pub total_blocks: u64,

    /// Size of the super block and the Reserved Area Number in blocks.
    pub rsvd_blocks: u32,

    /// Block size given by `bytes_per_block = 2.pow(block_size + 7)`.
    ///
    /// This value must be equal to or greater than 1.
    pub block_size: u8,

    /// Super-block checksum.
    ///
    /// The value of this field is such that the the sum of 8 lowest bytes of the field from
    /// [`magic`](struct.SuperBlock.html#structfield.magic) to
    /// [`block_size`](struct.SuperBlock.html#structfield.block_size), inclusive, is zero.
    pub crc: u8,
}

impl SuperBlock {
    /// Parse the super-block from the given device.
    ///
    /// # Errors
    ///
    /// Returns [`SfsError::BadMagic`] if the magic number found in the super-block is not equal to [`SFS_SIGNATURE`].
    ///
    ///  Returns [`SfsError::BadIndexAreaSize`] if the size of the Index Area is not a multiple of 64.
    ///
    /// Returns an [`Error::IO`] if the device could not be read.
    pub fn parse<Dev: Device>(celled_device: &Celled<Dev>) -> Result<Self, Error<SfsError>> {
        let mut device = celled_device.lock();

        let superblock = device.read_from_bytes::<Self>(Address::from(SUPER_BLOCK_START_BYTE), SUPER_BLOCK_SIZE)?;

        if superblock.crc != superblock.checksum_control() {
            Err(Error::Fs(FsError::Implementation(SfsError::BadChecksum {
                expected: superblock.checksum_control(),
                given: superblock.crc,
            })))
        } else if superblock.index_size.is_multiple_of(64) {
            Err(Error::Fs(FsError::Implementation(SfsError::BadIndexAreaSize(superblock.index_size))))
        } else if superblock.block_size == 0 {
            Err(Error::Fs(FsError::Implementation(SfsError::BadBlockSize(superblock.block_size))))
        } else {
            Ok(superblock)
        }
    }

    /// Returns the size in bytes of a block in this filesystem.
    #[must_use]
    pub const fn bytes_per_block(&self) -> u32 {
        1 << ((self.block_size as u32) + 7)
    }

    /// Returns the size in block of the Super Block Area.
    #[must_use]
    pub const fn superblock_area_size(&self) -> u32 {
        1
    }

    /// Returns the size in blocks of the Reserved Area.
    ///
    /// The block(s) that partially contain the [`SuperBlock`] are remove from the count.
    #[must_use]
    pub const fn reserved_area_size(&self) -> u32 {
        self.rsvd_blocks - self.superblock_area_size()
    }

    /// Returns the total size of the filesystem in bytes.
    #[must_use]
    pub const fn filesystem_size(&self) -> u64 {
        self.total_blocks * (self.bytes_per_block() as u64)
    }

    /// Returns the first block of the Index Area.
    #[must_use]
    pub const fn index_area_first_block(&self) -> u64 {
        self.total_blocks
            - (self.index_size / (self.bytes_per_block() as u64))
            - if self.index_size.is_multiple_of(self.bytes_per_block() as u64) { 0 } else { 1 }
    }

    /// Returns the complement to 0 checksum of the fields from [`magic`](struct.SuperBlock.html#structfield.magic) to
    /// [`block_size`](struct.SuperBlock.html#structfield.block_size), inclusive.
    #[must_use]
    pub const fn checksum_control(&self) -> u8 {
        let mut checksum = 0_u8;

        let mut i = 0;
        while i < 3 {
            checksum = checksum.wrapping_add(self.magic[i]);
            i += 1;
        }

        checksum = checksum.wrapping_add(self.version);

        let mut i = 0;
        while i < 8 {
            checksum = checksum.wrapping_add(((self.total_blocks >> (8 * i)) & 0xFF) as u8);
            i += 1;
        }

        let mut i = 0;
        while i < 4 {
            checksum = checksum.wrapping_add(((self.rsvd_blocks >> (8 * i)) & 0xFF) as u8);
            i += 1;
        }

        checksum = checksum.wrapping_add(self.block_size);

        0_u8.wrapping_sub(checksum)
    }

    /// Returns the area in which the block is located, or [`None`] if the block is outside the filesystem.
    #[must_use]
    pub const fn block_area(&self, block: u64) -> Option<Area> {
        if block < (self.superblock_area_size() as u64) {
            Some(Area::SuperBlock)
        } else if block < (self.rsvd_blocks as u64) {
            Some(Area::Reserved)
        } else if block < (self.rsvd_blocks as u64) + self.data_size {
            Some(Area::Data)
        } else if block < self.index_area_first_block() {
            Some(Area::Free)
        } else if block < self.total_blocks {
            Some(Area::Index)
        } else {
            None
        }
    }

    /// Checks whether the given block is located in the Data Area.
    #[must_use]
    pub const fn is_block_in_data_area(&self, block: u64) -> bool {
        matches!(self.block_area(block), Some(Area::Data))
    }

    /// Returns the [`Address`] of the starting byte of the Index Area.
    #[must_use]
    pub const fn index_area_starting_addr(&self) -> Address {
        let total_bytes = self.total_blocks * (self.bytes_per_block() as u64);
        Address::new(total_bytes - self.index_size)
    }
}

#[cfg(test)]
mod test {
    use spin::Lazy;

    use crate::fs::sfs::super_block::{Area, SuperBlock};
    use crate::fs::sfs::time_stamp::TimeStamp;

    static TEST_SUPER_BLOCK: Lazy<SuperBlock> = Lazy::new(|| SuperBlock {
        time_stamp: *TimeStamp::now().unwrap(),
        data_size: 5,
        index_size: 10 * 64,
        magic: *b"SFS",
        version: 0x10,
        total_blocks: 20,
        rsvd_blocks: 3,
        block_size: 1,
        crc: 0xEC,
    });

    #[test]
    fn super_block_values() {
        assert_eq!(TEST_SUPER_BLOCK.crc, TEST_SUPER_BLOCK.checksum_control());
        assert_eq!(TEST_SUPER_BLOCK.bytes_per_block(), 256);
        assert_eq!(TEST_SUPER_BLOCK.superblock_area_size(), 1);
        assert_eq!(TEST_SUPER_BLOCK.reserved_area_size(), 2);
        assert_eq!(TEST_SUPER_BLOCK.index_area_first_block(), 17);
    }

    #[test]
    fn blocks_areas() {
        assert_eq!(TEST_SUPER_BLOCK.block_area(0).unwrap(), Area::SuperBlock);
        assert_eq!(TEST_SUPER_BLOCK.block_area(1).unwrap(), Area::Reserved);
        assert_eq!(TEST_SUPER_BLOCK.block_area(2).unwrap(), Area::Reserved);
        assert_eq!(TEST_SUPER_BLOCK.block_area(3).unwrap(), Area::Data);
        assert_eq!(TEST_SUPER_BLOCK.block_area(7).unwrap(), Area::Data);
        assert_eq!(TEST_SUPER_BLOCK.block_area(8).unwrap(), Area::Free);
        assert_eq!(TEST_SUPER_BLOCK.block_area(16).unwrap(), Area::Free);
        assert_eq!(TEST_SUPER_BLOCK.block_area(17).unwrap(), Area::Index);
        assert_eq!(TEST_SUPER_BLOCK.block_area(19).unwrap(), Area::Index);
        assert_eq!(TEST_SUPER_BLOCK.block_area(20), None);
    }
}
