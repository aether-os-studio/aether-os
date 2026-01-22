//! Interface to manipulate blocks.
//!
//! A block is a contiguous part of the disk space. For a given filesystem, all the blocks have the same size, indicated
//! in the [`SuperBlock`](super::super_block::SuperBlock).
//!
//! See [its official specification](https://web.archive.org/web/20170315134201/https://www.d-rift.nl/combuster/vdisk/sfs.html#Super-Block) for more information.

use core::ops::Deref;

use super::SfsFs;
use crate::dev::Device;
use crate::fs::structures::block::BlockWrapper;

/// A sfs block.
///
/// The [`Device`] is splitted in contiguous ext2 blocks that have all the same size in bytes. This is **NOT** the block
/// as in block device, here "block" always refers to sfs' blocks. They start at 0, so the `n`th block will start at
/// the address `n * block_size`. Thus, a block is entirely described by its number.
#[derive(Clone)]
pub struct InnerBlock<Dev: Device> {
    /// Block number.
    number: u64,

    /// Sfs object associated with the device containing this block.
    filesystem: SfsFs<Dev>,
}

impl<Dev: Device> crate::fs::structures::block::Block<Dev> for InnerBlock<Dev> {
    type Num = u64;

    fn size(&self) -> u32 {
        self.filesystem.lock().super_block().bytes_per_block()
    }

    fn number(&self) -> Self::Num {
        self.number
    }

    fn device(&mut self) -> crate::celled::Celled<Dev> {
        let fs = self.filesystem.lock();
        fs.device.clone()
    }
}

/// Type alias to manipulate a sfs [`InnerBlock`] with [`Block`](crate::fs::structures::block::Block) trait.
pub type Block<Dev> = BlockWrapper<Dev, InnerBlock<Dev>>;

impl<Dev: Device> Block<Dev> {
    /// Returns a [`Block`] from its number and a [`SfsFs`] instance.
    #[must_use]
    pub const fn new(filesystem: SfsFs<Dev>, number: u64) -> Self {
        Self::new_wrapper(InnerBlock { number, filesystem })
    }

    /// Returns the number of the current block.
    #[must_use]
    pub const fn number(&self) -> u64 {
        self.deref().number
    }
}

impl<Dev: Device> From<Block<Dev>> for u64 {
    fn from(block: Block<Dev>) -> Self {
        block.number
    }
}
