//! Interface to manipulate blocks.
//!
//! A block is a contiguous part of the disk space. For a given filesystem, all the blocks have the same size, indicated
//! in the [`Superblock`].
//!
//! See [the OSDev wiki](https://wiki.osdev.org/Ext2#What_is_a_Block.3F) for more information.

use core::ops::Deref;

use super::Ext2Fs;
use super::error::Ext2Error;
use super::superblock::Superblock;
use crate::dev::Device;
use crate::error::Error;
use crate::fs::structures::block::BlockWrapper;

/// An ext2 block.
///
/// The [`Device`] is splitted in contiguous ext2 blocks that have all the same size in bytes. This is **NOT** the block
/// as in block device, here "block" always refers to ext2's blocks. They start at 0, so the `n`th block will start at
/// the address `n * block_size`. Thus, a block is entirely described by its number.
#[derive(Clone)]
pub struct InnerBlock<Dev: Device> {
    /// Block number.
    number: u32,

    /// Ext2 object associated with the device containing this block.
    filesystem: Ext2Fs<Dev>,
}

impl<Dev: Device> crate::fs::structures::block::Block<Dev> for InnerBlock<Dev> {
    type Num = u32;

    fn size(&self) -> u32 {
        self.filesystem.lock().superblock().block_size()
    }

    fn number(&self) -> Self::Num {
        self.number
    }

    fn device(&mut self) -> crate::celled::Celled<Dev> {
        let fs = self.filesystem.lock();
        fs.device.clone()
    }
}

/// Type alias to manipulate an ext2 [`InnerBlock`] with [`Block`](crate::fs::structures::block::Block) trait.
pub type Block<Dev> = BlockWrapper<Dev, InnerBlock<Dev>>;

impl<Dev: Device> Block<Dev> {
    /// Returns a [`Block`] from its number and an [`Ext2Fs`] instance.
    #[must_use]
    pub const fn new(filesystem: Ext2Fs<Dev>, number: u32) -> Self {
        Self::new_wrapper(InnerBlock { number, filesystem })
    }

    /// Returns the number of the current block.
    #[must_use]
    pub const fn number(&self) -> u32 {
        self.deref().number
    }

    /// Returns the containing block group of this block.
    #[must_use]
    pub const fn block_group(&self, superblock: &Superblock) -> u32 {
        superblock.block_group(self.number())
    }

    /// Returns the offset of this block in its containing block group.
    #[must_use]
    pub const fn group_index(&self, superblock: &Superblock) -> u32 {
        superblock.group_index(self.number())
    }

    /// Returns whether this block is currently free or not from the block bitmap in which the block resides.
    ///
    /// The `bitmap` argument is usually the result of the method
    /// [`get_block_bitmap`](../struct.Ext2.html#method.get_block_bitmap).
    #[allow(clippy::indexing_slicing)]
    #[must_use]
    pub const fn is_free(&self, superblock: &Superblock, bitmap: &[u8]) -> bool {
        let index = self.group_index(superblock) / 8;
        let offset = (self.number() - superblock.base().first_data_block) % 8;
        bitmap[index as usize] >> offset & 1 == 0
    }

    /// Returns whether this block is currently used or not from the block bitmap in which the block resides.
    ///
    /// The `bitmap` argument is usually the result of the method
    /// [`get_block_bitmap`](../struct.Ext2.html#method.get_block_bitmap).
    #[allow(clippy::indexing_slicing)]
    #[must_use]
    pub const fn is_used(&self, superblock: &Superblock, bitmap: &[u8]) -> bool {
        !self.is_free(superblock, bitmap)
    }

    /// Sets the current block usage in the block bitmap, and updates the superblock accordingly.
    ///
    /// # Errors
    ///
    /// Returns an [`BlockAlreadyInUse`](Ext2Error::BlockAlreadyInUse) error if the given block was already in use.
    ///
    /// Returns an [`BlockAlreadyFree`](Ext2Error::BlockAlreadyFree) error if the given block was already free.
    ///
    /// Returns an [`Error::IO`] if the device cannot be written.
    fn set_usage(&self, usage: bool) -> Result<(), Error<Ext2Error>> {
        self.filesystem.lock().locate_blocks(&[self.number], usage)
    }

    /// Sets the current block as free in the block bitmap, and updates the superblock accordingly.
    ///
    /// # Errors
    ///
    /// Returns an [`BlockAlreadyFree`](Ext2Error::BlockAlreadyFree) error if the given block was already free.
    ///
    /// Returns an [`Error::IO`] if the device cannot be written.
    pub fn set_free(&mut self) -> Result<(), Error<Ext2Error>> {
        self.set_usage(false)
    }

    /// Sets the current block as used in the block bitmap, and updates the superblock accordingly.
    ///
    /// # Errors
    ///
    /// Returns an [`BlockAlreadyInUse`](Ext2Error::BlockAlreadyInUse) error if the given block was already in use.
    ///
    /// Returns an [`Error::IO`] if the device cannot be written.
    pub fn set_used(&mut self) -> Result<(), Error<Ext2Error>> {
        self.set_usage(true)
    }
}

impl<Dev: Device> From<Block<Dev>> for u32 {
    fn from(block: Block<Dev>) -> Self {
        block.number
    }
}

#[cfg(test)]
mod test {
    use alloc::vec;
    use std::fs::File;

    use deku::no_std_io::{Read, Seek, SeekFrom, Write};

    use crate::celled::Celled;
    use crate::dev::Device;
    use crate::dev::address::Address;
    use crate::fs::ext2::Ext2Fs;
    use crate::fs::ext2::block::Block;
    use crate::fs::ext2::block_group::BlockGroupDescriptor;
    use crate::fs::ext2::superblock::Superblock;
    use crate::tests::new_device_id;

    fn block_read(file: File) {
        const BLOCK_NUMBER: u32 = 2;

        let celled_file = Celled::new(file);
        let superblock = Superblock::parse(&celled_file).unwrap();

        let block_starting_addr = Address::new((BLOCK_NUMBER * superblock.block_size()).into());
        let slice =
            <File as Device>::slice(&mut celled_file.lock(), block_starting_addr + 123..block_starting_addr + 123 + 59)
                .unwrap()
                .commit();

        let ext2 = Ext2Fs::new_celled(celled_file, new_device_id()).unwrap();
        let mut block = Block::new(ext2, BLOCK_NUMBER);
        block.seek(SeekFrom::Start(123)).unwrap();
        let mut buffer_auto = [0_u8; 59];
        block.read(&mut buffer_auto).unwrap();

        assert_eq!(buffer_auto, slice.as_ref());
    }

    fn block_write(file: File) {
        const BLOCK_NUMBER: u32 = 10_234;

        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let superblock = ext2.lock().superblock().clone();

        let mut block = Block::new(ext2, BLOCK_NUMBER);
        let mut buffer = vec![0_u8; usize::try_from(superblock.block_size()).unwrap() - 123];
        buffer[..59].copy_from_slice(&[1_u8; 59]);
        block.seek(SeekFrom::Start(123)).unwrap();
        block.write(&buffer).unwrap();

        let mut start = vec![0_u8; 123];
        start.append(&mut buffer);

        let mut block_content = vec![0_u8; 1024];
        std::println!("{}", block_content.len());
        block.seek(SeekFrom::Start(0)).unwrap();
        block.read(&mut block_content).unwrap();
        assert_eq!(block_content, start);
    }

    fn block_set_free(file: File) {
        // This block should not be free
        const BLOCK_NUMBER: u32 = 9;

        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let superblock = ext2.lock().superblock().clone();

        let mut block = Block::new(ext2.clone(), BLOCK_NUMBER);
        let block_group = block.block_group(&superblock);

        let fs = ext2.lock();
        let block_group_descriptor = BlockGroupDescriptor::parse(&fs, block_group).unwrap();
        let free_block_count = block_group_descriptor.free_blocks_count;

        let bitmap = fs.get_block_bitmap(block_group).unwrap();

        drop(fs);

        assert!(block.is_used(&superblock, &bitmap));

        block.set_free().unwrap();

        let fs = ext2.lock();
        let new_free_block_count = BlockGroupDescriptor::parse(&fs, block.block_group(&superblock))
            .unwrap()
            .free_blocks_count;

        assert!(block.is_free(&superblock, &fs.get_block_bitmap(block_group).unwrap()));
        assert_eq!(free_block_count + 1, new_free_block_count);
    }

    fn block_set_used(file: File) {
        // This block should not be used
        const BLOCK_NUMBER: u32 = 1920;

        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let superblock = ext2.lock().superblock().clone();

        let mut block = Block::new(ext2.clone(), BLOCK_NUMBER);
        let block_group = block.block_group(&superblock);

        let fs = ext2.lock();

        let block_group_descriptor = BlockGroupDescriptor::parse(&fs, block_group).unwrap();
        let free_block_count = block_group_descriptor.free_blocks_count;

        let bitmap = fs.get_block_bitmap(block_group).unwrap();

        assert!(block.is_free(&superblock, &bitmap));

        drop(fs);

        block.set_used().unwrap();

        let fs = ext2.lock();
        let new_free_block_count = BlockGroupDescriptor::parse(&fs, block.block_group(&superblock))
            .unwrap()
            .free_blocks_count;

        assert!(block.is_used(&superblock, &fs.get_block_bitmap(block_group).unwrap()));
        assert_eq!(free_block_count - 1, new_free_block_count);
    }

    mod generated {
        use crate::tests::{PostCheck, generate_fs_test};

        generate_fs_test!(block_read, "./tests/fs/ext2/io_operations.ext2", PostCheck::Ext);
        generate_fs_test!(block_write, "./tests/fs/ext2/io_operations.ext2", PostCheck::Ext);

        // Unsound changes on the ext2 filesystem are made so there should not be a e2fsck check afterward.
        generate_fs_test!(block_set_free, "./tests/fs/ext2/io_operations.ext2", PostCheck::None);
        generate_fs_test!(block_set_used, "./tests/fs/ext2/io_operations.ext2", PostCheck::None);
    }
}
