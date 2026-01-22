//! Interface with ext2's block group descriptors and block group descriptor table.
//!
//! See the [OSdev wiki](https://wiki.osdev.org/Ext2#Block_Group_Descriptor_Table) and the [*The Second Extended Filesystem* book](https://www.nongnu.org/ext2-doc/ext2.html) for more information.

use deku::{DekuRead, DekuWrite};

use super::Ext2;
use super::error::Ext2Error;
use super::superblock::Superblock;
use crate::dev::Device;
use crate::dev::address::Address;
use crate::error::Error;
use crate::fs::error::FsError;

/// Size in bytes of a block group descriptor with reserved bytes.
pub const BLOCK_GROUP_DESCRIPTOR_SIZE: usize = 32;

/// Block group descriptor.
///
/// Contains information regarding where important data structures for that block group are located.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(endian = "little")]
#[cfg_attr(test, derive(PartialEq, Eq))]
#[allow(clippy::module_name_repetitions)]
pub struct BlockGroupDescriptor {
    /// Block address of block usage bitmap.
    pub block_bitmap: u32,

    /// Block address of inode usage bitmap.
    pub inode_bitmap: u32,

    /// Starting block address of inode table.
    pub inode_table: u32,

    /// Number of unallocated blocks in group.
    pub free_blocks_count: u16,

    /// Number of unallocated inodes in group.
    pub free_inodes_count: u16,

    /// Number of directories in group.
    pub used_dirs_count: u16,

    /// Used for padding the structure on a 32bit boundary.
    pub pad: u16,

    /// Reserved space for future revisions.
    pub reserved: [u8; 12],
}

impl BlockGroupDescriptor {
    /// Returns the starting address of the `n`th block group descriptor (starting at 0).
    ///
    /// # Errors
    ///
    /// Returns an [`NonExistingBlockGroup`](Ext2Error::NonExistingBlockGroup) if `n` is greater than the block group
    /// count of this device.
    pub const fn starting_addr(superblock: &Superblock, n: u32) -> Result<Address, Error<Ext2Error>> {
        let block_group_count = superblock.block_group_count();
        if block_group_count <= n {
            return Err(Error::Fs(FsError::Implementation(Ext2Error::NonExistingBlockGroup(n))));
        }

        let block_size = superblock.block_size() as u64;
        
        // FIX: BGDT 总是从 superblock 所在 block 的下一个 block 开始
        // - block_size == 1024: superblock 在 block 1, BGDT 从 block 2 开始
        // - block_size > 1024:  superblock 在 block 0, BGDT 从 block 1 开始
        let bgdt_start_block: u64 = if block_size == 1024 { 2 } else { 1 };
        let bgdt_start_address = bgdt_start_block * block_size;
        
        Ok(Address::new(bgdt_start_address + BLOCK_GROUP_DESCRIPTOR_SIZE as u64 * (n as u64)))
    }

    /// Parse the `n`th block group descriptor from the given device (starting at 0).
    ///
    /// # Errors
    ///
    /// Returns an [`NonExistingBlockGroup`](Ext2Error::NonExistingBlockGroup) if `n` is greater than the block group
    /// count of this device.
    ///
    /// Returns an [`Error::IO`] if the device cannot be read.
    pub fn parse<Dev: Device>(fs: &Ext2<Dev>, n: u32) -> Result<Self, Error<Ext2Error>> {
        let mut device = fs.device.lock();

        let block_group_descriptor_address = Self::starting_addr(fs.superblock(), n)?;

        let block_group_descriptor =
            device.read_from_bytes::<Self>(block_group_descriptor_address, BLOCK_GROUP_DESCRIPTOR_SIZE)?;

        Ok(block_group_descriptor)
    }

    /// Writes the given `block_group_descriptor` structure at its position.
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device cannot be written.
    ///
    /// # Safety
    ///
    /// The given `block_group_descriptor` must correspond to the given inode number `n`.
    pub(crate) unsafe fn write_on_device<Dev: Device>(
        fs: &Ext2<Dev>,
        n: u32,
        block_group_descriptor: Self,
    ) -> Result<(), Error<Ext2Error>> {
        let starting_addr = Self::starting_addr(fs.superblock(), n)?;
        fs.device.lock().write_to_bytes(starting_addr, block_group_descriptor).map_err(Into::into)
    }
}

#[cfg(test)]
mod test {
    use core::mem::size_of;
    use std::fs::File;

    use super::{BLOCK_GROUP_DESCRIPTOR_SIZE, BlockGroupDescriptor};
    use crate::fs::ext2::Ext2;
    use crate::tests::new_device_id;

    #[test]
    fn struct_size() {
        assert_eq!(size_of::<BlockGroupDescriptor>(), BLOCK_GROUP_DESCRIPTOR_SIZE);
    }

    fn parse_first_block_group_descriptor_base(file: File) {
        let fs = Ext2::new(file, new_device_id()).unwrap();
        assert!(BlockGroupDescriptor::parse(&fs, 0).is_ok());
    }

    fn parse_first_block_group_descriptor_extended(file: File) {
        let fs = Ext2::new(file, new_device_id()).unwrap();
        assert!(BlockGroupDescriptor::parse(&fs, 0).is_ok());
    }

    fn failed_parse_base(file: File) {
        let fs = Ext2::new(file, new_device_id()).unwrap();
        assert!(BlockGroupDescriptor::parse(&fs, fs.superblock().block_group_count()).is_err());
    }

    fn failed_parse_extended(file: File) {
        let fs = Ext2::new(file, new_device_id()).unwrap();
        assert!(BlockGroupDescriptor::parse(&fs, fs.superblock().block_group_count()).is_err());
    }

    fn write_back(file: File) {
        let fs = Ext2::new(file, new_device_id()).unwrap();

        let mut bgd = BlockGroupDescriptor::parse(&fs, 0).unwrap();
        bgd.free_blocks_count = 0;
        bgd.reserved = [0x9A; 12];
        unsafe { BlockGroupDescriptor::write_on_device(&fs, 0, bgd).unwrap() };

        let new_bgd = BlockGroupDescriptor::parse(&fs, 0).unwrap();
        assert_eq!(bgd, new_bgd);
    }

    mod generated {
        use crate::tests::{PostCheck, generate_fs_test};

        generate_fs_test!(parse_first_block_group_descriptor_base, "./tests/fs/ext2/base.ext2", PostCheck::Ext);
        generate_fs_test!(parse_first_block_group_descriptor_extended, "./tests/fs/ext2/extended.ext2", PostCheck::Ext);
        generate_fs_test!(failed_parse_base, "./tests/fs/ext2/base.ext2", PostCheck::Ext);
        generate_fs_test!(failed_parse_extended, "./tests/fs/ext2/extended.ext2", PostCheck::Ext);

        // Unsound changes on the ext2 filesystem are made so there should not be a e2fsck check afterward.
        generate_fs_test!(write_back, "./tests/fs/ext2/io_operations.ext2", PostCheck::None);
    }
}
