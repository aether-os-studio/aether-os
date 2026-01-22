//! Interface to manipulate UNIX files on an ext2 filesystem.

use alloc::borrow::ToOwned;
use alloc::ffi::CString;
use alloc::string::{String, ToString};
use alloc::vec;
use alloc::vec::Vec;
use core::fmt::Debug;
use core::ptr::{addr_of, addr_of_mut, slice_from_raw_parts};

use bitflags::Flags;
use deku::no_std_io::{Read, Seek, SeekFrom, Write};
use itertools::Itertools;
use spin::Mutex;

use super::Ext2Fs;
use super::directory::{self, Entry, FileType};
use super::error::Ext2Error;
use super::inode::{Inode, TypePermissions};
use crate::arch::{u32_to_usize, u64_to_usize, usize_to_u64};
use crate::dev::Device;
use crate::dev::address::Address;
use crate::error::Error;
use crate::fs::PATH_MAX;
use crate::fs::error::FsError;
use crate::fs::ext2::block::Block;
use crate::fs::ext2::inode::DIRECT_BLOCK_POINTER_COUNT;
use crate::fs::file::{self, DirectoryEntry, DirectoryRead, Stat, Type, TypeWithFile};
use crate::fs::permissions::Permissions;
use crate::fs::structures::indirection::IndirectedBlocks;
use crate::fs::types::{Blkcnt, Blksize, Gid, Ino, Mode, Nlink, Off, Time, Timespec, Uid};
use crate::path::{CUR_DIR, PARENT_DIR, UnixStr};

/// Limit in bytes for the length of a pointed path of a symbolic link to be store in an inode and not in a separate
/// data block.
pub const SYMBOLIC_LINK_INODE_STORE_LIMIT: usize = 60;

/// General file implementation.
pub struct File<Dev: Device> {
    /// Ext2 object associated with the device containing this file.
    filesystem: Ext2Fs<Dev>,

    /// Inode number of the inode corresponding to the file.
    inode_number: u32,

    /// Inode corresponding to the file.
    inode: Inode,

    /// Read/Write offset in bytes (can be manipulated with [`Seek`]).
    io_offset: u64,
}

impl<Dev: Device> Debug for File<Dev> {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter
            .debug_struct("File")
            .field("device_id", &self.filesystem.lock().device_id)
            .field("inode_number", &self.inode_number)
            .field("inode", &self.inode)
            .field("io_offset", &self.io_offset)
            .finish()
    }
}

impl<Dev: Device> Clone for File<Dev> {
    fn clone(&self) -> Self {
        Self {
            filesystem: self.filesystem.clone(),
            inode_number: self.inode_number,
            inode: self.inode,
            io_offset: self.io_offset,
        }
    }
}

impl<Dev: Device> File<Dev> {
    /// Returns a new ext2's [`File`] from an [`Ext2Fs`] instance and the inode number of the file.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Inode::parse`].
    pub fn new(filesystem: &Ext2Fs<Dev>, inode_number: u32) -> Result<Self, Error<Ext2Error>> {
        let fs = filesystem.lock();
        let inode = Inode::parse(&fs, inode_number)?;
        Ok(Self {
            filesystem: filesystem.clone(),
            inode_number,
            inode,
            io_offset: 0,
        })
    }

    /// Updates the inner [`Inode`].
    fn update_inner_inode(&mut self) -> Result<(), Error<Ext2Error>> {
        let fs = self.filesystem.lock();
        self.inode = Inode::parse(&fs, self.inode_number)?;
        Ok(())
    }

    ///  Sets the file's inode to the given object.
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device cannot be written.
    ///
    /// # Safety
    ///
    /// Must ensure that the given inode is coherent with the current state of the filesystem.
    unsafe fn set_inode(&mut self, inode: &Inode) -> Result<(), Error<Ext2Error>> {
        let fs = self.filesystem.lock();
        unsafe { Inode::write_on_device(&fs, self.inode_number, *inode) }?;
        drop(fs);

        self.update_inner_inode()?;

        Ok(())
    }

    /// General implementation of [`truncate`](file::Regular::truncate) for ext2's [`File`].
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`truncate`](file::Regular::truncate).
    pub fn truncate(&mut self, size: u64) -> Result<(), Error<Ext2Error>> {
        if self.inode.data_size() <= size {
            return Ok(());
        }

        let mut fs = self.filesystem.lock();

        let mut new_inode = self.inode;
        // SAFETY: the result is smaller than `u32::MAX`
        new_inode.size = unsafe { u32::try_from(u64::from(u32::MAX) & size).unwrap_unchecked() };

        let time = fs.get_time();
        new_inode.atime = time;
        new_inode.mtime = time;

        let kept_data_blocks_number = if size == 0 {
            0
        } else {
            // SAFETY: the result is a u32 as `size` is valid (it has been checked)
            unsafe {
                1 + u32::try_from(size.saturating_sub(1) / u64::from(fs.superblock().block_size())).unwrap_unchecked()
            }
        };
        let indirection_blocks = self.inode.indirected_blocks(&fs)?;

        let mut new_indirection_blocks = indirection_blocks.clone();
        new_indirection_blocks.truncate_back_data_blocks(kept_data_blocks_number);

        new_inode.blocks = (new_indirection_blocks.data_block_count()
            + new_indirection_blocks.indirection_block_count())
            * fs.superblock().block_size()
            / 512;

        let mut direct_block_pointers = new_inode.direct_block_pointers;
        for i in 0..u32_to_usize(DIRECT_BLOCK_POINTER_COUNT) {
            // SAFETY: there is exactly `DIRECT_BLOCK_POINTER_COUNT` direct block pointers in an inode
            let block = unsafe { direct_block_pointers.get_mut(i).unwrap_unchecked() };
            *block = new_indirection_blocks.direct_blocks.get(i).copied().unwrap_or_default();
        }
        new_inode.direct_block_pointers = direct_block_pointers;
        new_inode.singly_indirect_block_pointer = new_indirection_blocks.singly_indirected_blocks.0;
        new_inode.doubly_indirect_block_pointer = new_indirection_blocks.doubly_indirected_blocks.0;
        new_inode.triply_indirect_block_pointer = new_indirection_blocks.triply_indirected_blocks.0;

        let symmetrical_difference = indirection_blocks.truncate_front_data_blocks(kept_data_blocks_number);

        let mut deallocated_blocks = symmetrical_difference.changed_data_blocks();
        deallocated_blocks.append(
            &mut symmetrical_difference
                .changed_indirected_blocks()
                .into_iter()
                .map(|(_, (indirection_block, _))| indirection_block)
                .collect_vec(),
        );

        // SAFETY: this writes an inode at the starting address of the inode
        unsafe {
            Inode::write_on_device(&fs, self.inode_number, new_inode)?;
        };

        fs.deallocate_blocks(&deallocated_blocks)?;

        drop(fs);

        self.io_offset = 0;

        self.update_inner_inode()
    }

    /// Reads all the content of the file and returns it in a byte vector.
    ///
    /// Does not move the offset for I/O operations used by [`Seek`].
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Inode::read_data`].
    pub fn read_all(&mut self) -> Result<Vec<u8>, Error<Ext2Error>> {
        let mut buffer = vec![0_u8; u64_to_usize(self.inode.data_size()).map_err(Error::from_infallible)?];
        let previous_offset = self.seek(SeekFrom::Start(0))?;
        self.read_exact(&mut buffer)?;
        self.seek(SeekFrom::Start(previous_offset))?;
        Ok(buffer)
    }
}

impl<Dev: Device> file::Base for File<Dev> {
    type FsError = Ext2Error;
}

impl<Dev: Device> file::FileRead for File<Dev> {
    fn stat(&self) -> file::Stat {
        let filesystem = self.filesystem.lock();

        Stat {
            dev: crate::fs::types::Dev(filesystem.device_id),
            ino: Ino(u64::from(self.inode_number)),
            mode: Mode(self.inode.mode),
            nlink: Nlink(u32::from(self.inode.links_count)),
            uid: Uid(self.inode.uid.into()),
            gid: Gid(self.inode.gid.into()),
            rdev: crate::fs::types::Dev::default(),
            size: Off(self.inode.data_size().try_into().unwrap_or_default()),
            atim: Timespec {
                tv_sec: Time(self.inode.atime.into()),
                tv_nsec: u32::default(),
            },
            mtim: Timespec {
                tv_sec: Time(self.inode.mtime.into()),
                tv_nsec: u32::default(),
            },
            ctim: Timespec {
                tv_sec: Time(self.inode.ctime.into()),
                tv_nsec: u32::default(),
            },
            // SAFETY: it is safe to assume that `block_size << isize::MAX` with `isize` at least `i32`
            blksize: Blksize(unsafe { u32_to_usize(filesystem.superblock.block_size()).try_into().unwrap_unchecked() }),
            blkcnt: Blkcnt(self.inode.blocks.into()),
        }
    }

    fn get_type(&self) -> file::Type {
        self.inode.file_type().unwrap_or_else(|_| {
            panic!(
                "The inner inode with number {} is in an incoherent state: its file type is not valid",
                self.inode_number
            )
        })
    }
}

impl<Dev: Device> file::File for File<Dev> {
    fn set_mode(&mut self, mode: Mode) -> Result<(), Error<Self::FsError>> {
        let mut new_inode = self.inode;
        new_inode.mode = *mode | self.inode.type_permissions().file_type().bits();
        // SAFETY: only the mode has changed
        unsafe { self.set_inode(&new_inode) }
    }

    fn set_uid(&mut self, uid: Uid) -> Result<(), Error<Self::FsError>> {
        let mut new_inode = self.inode;
        new_inode.uid = TryInto::<u16>::try_into(uid.0)
            .map_err(|_| Error::Fs(FsError::Implementation(Ext2Error::UidTooLarge(uid.0))))?;
        // SAFETY: only the UID has changed
        unsafe { self.set_inode(&new_inode) }
    }

    fn set_gid(&mut self, gid: Gid) -> Result<(), Error<Self::FsError>> {
        let mut new_inode = self.inode;
        new_inode.gid = TryInto::<u16>::try_into(gid.0)
            .map_err(|_| Error::Fs(FsError::Implementation(Ext2Error::GidTooLarge(gid.0))))?;
        // SAFETY: only the GID has changed
        unsafe { self.set_inode(&new_inode) }
    }

    fn set_atim(&mut self, atim: Timespec) -> Result<(), Error<Self::FsError>> {
        let mut new_inode = self.inode;
        // SAFETY: `X % i64::from(u32::MAX) < u32::MAX`
        new_inode.atime = unsafe { u32::try_from(*atim.tv_sec % i64::from(u32::MAX)).unwrap_unchecked() };
        // SAFETY: only the atime has changed
        unsafe { self.set_inode(&new_inode) }
    }

    fn set_mtim(&mut self, mtim: Timespec) -> Result<(), Error<Self::FsError>> {
        let mut new_inode = self.inode;
        // SAFETY: `X % i64::from(u32::MAX) < u32::MAX`
        new_inode.mtime = unsafe { u32::try_from(*mtim.tv_sec % i64::from(u32::MAX)).unwrap_unchecked() };
        // SAFETY: only the mtime has changed
        unsafe { self.set_inode(&new_inode) }
    }

    fn set_ctim(&mut self, ctim: Timespec) -> Result<(), Error<Self::FsError>> {
        let mut new_inode = self.inode;
        // SAFETY: `X % i64::from(u32::MAX) < u32::MAX`
        new_inode.ctime = unsafe { u32::try_from(*ctim.tv_sec % i64::from(u32::MAX)).unwrap_unchecked() };
        // SAFETY: only the ctime has changed
        unsafe { self.set_inode(&new_inode) }
    }
}

macro_rules! impl_file {
    ($id:ident) => {
        impl<Dev: Device> crate::fs::file::Base for $id<Dev> {
            type FsError = Ext2Error;
        }

        impl<Dev: Device> crate::fs::file::FileRead for $id<Dev> {
            fn stat(&self) -> Stat {
                self.file.stat()
            }

            fn get_type(&self) -> file::Type {
                self.file.get_type()
            }
        }

        impl<Dev: Device> crate::fs::file::File for $id<Dev> {
            fn set_mode(&mut self, mode: Mode) -> Result<(), Error<Self::FsError>> {
                self.file.set_mode(mode)
            }

            fn set_uid(&mut self, uid: Uid) -> Result<(), Error<Self::FsError>> {
                self.file.set_uid(uid)
            }

            fn set_gid(&mut self, gid: Gid) -> Result<(), Error<Self::FsError>> {
                self.file.set_gid(gid)
            }

            fn set_atim(&mut self, atim: Timespec) -> Result<(), Error<Self::FsError>> {
                self.file.set_atim(atim)
            }

            fn set_mtim(&mut self, mtim: Timespec) -> Result<(), Error<Self::FsError>> {
                self.file.set_mtim(mtim)
            }

            fn set_ctim(&mut self, ctim: Timespec) -> Result<(), Error<Self::FsError>> {
                self.file.set_ctim(ctim)
            }
        }
    };
}

impl<Dev: Device> Read for File<Dev> {
    fn read(&mut self, buf: &mut [u8]) -> deku::no_std_io::Result<usize> {
        let filesystem = self.filesystem.lock();
        let bytes = self
            .inode
            .read_data(&filesystem, buf, self.io_offset)
            .inspect(|&bytes| {
                self.io_offset += usize_to_u64(bytes);
            })
            .map_err(|err| deku::no_std_io::Error::new(deku::no_std_io::ErrorKind::InvalidData, err.to_string()))?;

        let mut device = filesystem.device.lock();
        if let Some(now) = device.now() {
            drop(device);

            let mut new_inode = self.inode;
            // SAFETY: the result will always be under u32::MAX
            new_inode.atime = unsafe { (now.tv_sec.0 & i64::from(u32::MAX)).try_into().unwrap_unchecked() };
            // SAFETY: only the access time has been updated
            unsafe {
                Inode::write_on_device(&filesystem, self.inode_number, new_inode).map_err(|err| {
                    deku::no_std_io::Error::new(deku::no_std_io::ErrorKind::InvalidData, err.to_string())
                })?;
            };
        }

        Ok(bytes)
    }
}

impl<Dev: Device> Write for File<Dev> {
    #[allow(clippy::too_many_lines)]
    fn write(&mut self, buf: &[u8]) -> deku::no_std_io::Result<usize> {
        let mut fs = self.filesystem.lock();
        let superblock = fs.superblock().clone();
        let block_size = u64::from(fs.superblock().block_size());

        let buf_len = usize_to_u64(buf.len());
        if buf_len > fs.superblock().max_file_size() {
            return Err(deku::no_std_io::Error::new(
                deku::no_std_io::ErrorKind::InvalidInput,
                "Tried to read a file from a buffer with a length greater than the max file length",
            ));
        }

        // Calcul of the number of needed data blocks
        let bytes_to_write = buf_len;
        let data_blocks_needed =
            // SAFETY: there are at most u32::MAX blocks on the filesystem
            1 + unsafe { u32::try_from((bytes_to_write + self.io_offset - 1) / block_size).unwrap_unchecked() };

        if !fs.options().large_files && u64::from(data_blocks_needed) * block_size >= u64::from(u32::MAX) {
            return Err(deku::no_std_io::Error::new(
                deku::no_std_io::ErrorKind::InvalidInput,
                "Tried to write a large file while the filesystem does not have the required feature set",
            ));
        }

        let mut indirected_blocks = self.inode.indirected_blocks(&fs)?;
        // SAFETY: there are at most u32::MAX blocks on the filesystem
        indirected_blocks.truncate_back_data_blocks(unsafe {
            // In case of blocks that are not used and not 0
            1 + u32::try_from((self.inode.data_size().max(1) - 1) / block_size).unwrap_unchecked()
        });

        let current_data_block_count = indirected_blocks.data_block_count();
        let data_blocks_to_request = data_blocks_needed.saturating_sub(current_data_block_count);

        let current_indirection_block_count = indirected_blocks.indirection_block_count();
        let indirection_blocks_to_request =
            IndirectedBlocks::<DIRECT_BLOCK_POINTER_COUNT>::necessary_indirection_block_count(
                data_blocks_needed,
                fs.superblock().base().block_size() / 4,
            ) - current_indirection_block_count;

        let start_block_group = indirected_blocks
            .last_data_block_allocated()
            .map(|(block, _)| superblock.block_group(block))
            .unwrap_or_default();

        let free_blocks =
            fs.free_blocks_offset(data_blocks_to_request + indirection_blocks_to_request, start_block_group)?;

        fs.allocate_blocks(&free_blocks)?;

        drop(fs);

        let (new_indirected_blocks, changed_blocks) = indirected_blocks.append_blocks_with_difference(
            &free_blocks,
            // SAFETY: this result points to a block which is encoded on 32 bits
            Some(unsafe { u32::try_from(self.io_offset / u64::from(superblock.block_size())).unwrap_unchecked() }),
        );

        for (starting_index, (indirection_block, blocks)) in changed_blocks.changed_indirected_blocks() {
            let mut block = Block::new(self.filesystem.clone(), indirection_block);
            if starting_index != 0 {
                block.seek(SeekFrom::Start(usize_to_u64(starting_index)))?;
            }

            // SAFETY: it is always possible to cast a u32 to 4 u8
            block.write_all(unsafe { &*slice_from_raw_parts(blocks.as_ptr().cast::<u8>(), blocks.len() * 4) })?;
        }

        let mut written_bytes = 0_usize;

        let changed_data_blocks = changed_blocks.changed_data_blocks();
        let changed_data_blocks_iterator = &mut changed_data_blocks.iter();

        if let Some(block_number) = changed_data_blocks_iterator.next() {
            let mut block = Block::new(self.filesystem.clone(), *block_number);
            block.seek(SeekFrom::Start(self.io_offset % u64::from(superblock.block_size())))?;
            written_bytes += block.write(buf)?;
        }

        for block_number in changed_data_blocks_iterator {
            let mut block = Block::new(self.filesystem.clone(), *block_number);
            let Some(buffer_end) = buf.get(
                written_bytes
                    ..written_bytes
                        + u32_to_usize(superblock.base().blocks_per_group)
                            .min(u64_to_usize(buf_len).map_err(Error::<Ext2Error>::from_infallible)? - written_bytes),
            ) else {
                break;
            };

            let new_written_bytes = block.write(buffer_end)?;
            if new_written_bytes == 0 {
                break;
            }
            written_bytes += new_written_bytes;
        }

        let mut updated_inode = self.inode;

        let total_block_used =
            new_indirected_blocks.data_block_count() + new_indirected_blocks.indirection_block_count();
        let (
            mut direct_block_pointers,
            singly_indirected_block_pointer,
            doubly_indirected_block_pointer,
            triply_indirected_block_pointer,
        ) = new_indirected_blocks.blocks();

        direct_block_pointers
            .append(&mut vec![0_u32; 12].into_iter().take(12 - direct_block_pointers.len()).collect_vec());

        let mut updated_direct_block_pointers = updated_inode.direct_block_pointers;
        updated_direct_block_pointers.clone_from_slice(&direct_block_pointers);
        updated_inode.direct_block_pointers = updated_direct_block_pointers;

        updated_inode.singly_indirect_block_pointer = singly_indirected_block_pointer.0;
        updated_inode.doubly_indirect_block_pointer = doubly_indirected_block_pointer.0;
        updated_inode.triply_indirect_block_pointer = triply_indirected_block_pointer.0;

        let new_size = self.inode.data_size().max(self.io_offset + buf_len);

        // SAFETY: the result cannot be greater than `u32::MAX`
        updated_inode.size = unsafe { u32::try_from(new_size & u64::from(u32::MAX)).unwrap_unchecked() };
        updated_inode.blocks = (total_block_used * self.filesystem.lock().superblock().block_size()) / 512;

        // SAFETY: the result cannot be greater than `u32::MAX`
        updated_inode.dir_acl = unsafe { u32::try_from((new_size >> 32) & u64::from(u32::MAX)).unwrap_unchecked() };

        let fs = self.filesystem.lock();

        let time = fs.get_time();
        updated_inode.atime = time;
        updated_inode.mtime = time;

        drop(fs);

        // SAFETY: the updated inode contains the right inode created in this function
        unsafe { self.set_inode(&updated_inode) }?;

        self.seek(SeekFrom::Current(i64::try_from(buf_len).expect("Could not fit the buffer length on an i64")))?;

        Ok(written_bytes)
    }

    fn flush(&mut self) -> deku::no_std_io::Result<()> {
        Ok(())
    }
}

impl<Dev: Device> Seek for File<Dev> {
    fn seek(&mut self, pos: SeekFrom) -> deku::no_std_io::Result<u64> {
        // SAFETY: it is safe to assume that the file length is smaller than 2^63 bytes long
        let file_length = unsafe { i64::try_from(self.inode.data_size()).unwrap_unchecked() };

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
                self.io_offset = (unsafe { TryInto::<i64>::try_into(previous_offset).unwrap_unchecked() } + add_offset)
                    .try_into()
                    .map_err(|_err| {
                        deku::no_std_io::Error::new(
                            deku::no_std_io::ErrorKind::InvalidInput,
                            "Invalid seek to a negative or overflowing position",
                        )
                    })?;
            },
        }

        if self.io_offset > self.inode.data_size() {
            Err(deku::no_std_io::Error::new(
                deku::no_std_io::ErrorKind::InvalidInput,
                "Invalid seek to a negative or overflowing position",
            ))
        } else {
            Ok(previous_offset)
        }
    }
}

/// Implementation of a regular file.
#[derive(Debug)]
pub struct Regular<Dev: Device> {
    /// Inner file containing the generic file.
    file: File<Dev>,
}

impl<Dev: Device> Regular<Dev> {
    /// Returns a new ext2's [`Regular`] from an [`Ext2Fs`] instance and the inode number of the file.
    ///
    /// # Errors
    ///
    /// Returns the same errors  as [`Ext2::inode`](super::Ext2::inode).
    pub fn new(filesystem: &Ext2Fs<Dev>, inode_number: u32) -> Result<Self, Error<Ext2Error>> {
        Ok(Self {
            file: File::new(&filesystem.clone(), inode_number)?,
        })
    }

    /// Reads all the content of the file and returns it in a byte vector.
    ///
    /// Does not move the offset for I/O operations used by [`Seek`].
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Inode::read_data`].
    pub fn read_all(&mut self) -> Result<Vec<u8>, Error<Ext2Error>> {
        self.file.read_all()
    }
}

impl<Dev: Device> Clone for Regular<Dev> {
    fn clone(&self) -> Self {
        Self {
            file: self.file.clone(),
        }
    }
}

impl_file!(Regular);

impl<Dev: Device> Read for Regular<Dev> {
    fn read(&mut self, buf: &mut [u8]) -> deku::no_std_io::Result<usize> {
        self.file.read(buf)
    }
}

impl<Dev: Device> Write for Regular<Dev> {
    fn write(&mut self, buf: &[u8]) -> deku::no_std_io::Result<usize> {
        self.file.write(buf)
    }

    fn flush(&mut self) -> deku::no_std_io::Result<()> {
        self.file.flush()
    }
}

impl<Dev: Device> Seek for Regular<Dev> {
    fn seek(&mut self, pos: SeekFrom) -> deku::no_std_io::Result<u64> {
        self.file.seek(pos)
    }
}

impl<Dev: Device> file::RegularRead for Regular<Dev> {}

impl<Dev: Device> file::Regular for Regular<Dev> {
    fn truncate(&mut self, size: u64) -> Result<(), Error<Self::FsError>> {
        self.file.truncate(size)
    }
}

/// Interface for ext2's directories.
///
/// In ext2, the content of a directory is a list of [`Entry`], which are the children of the directory. In particular,
/// `.` and `..` are always children of a directory.
#[derive(Debug)]
pub struct Directory<Dev: Device> {
    /// Inner file containing the generic file.
    file: File<Dev>,

    /// Entries contained in this directory.
    ///
    /// They are stored as a list of entries in each data block.
    entries: Mutex<Vec<Vec<Entry>>>,
}

impl<Dev: Device> Directory<Dev> {
    /// Returns the directory located at the given inode number.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Entry::parse`].
    pub fn new(filesystem: &Ext2Fs<Dev>, inode_number: u32) -> Result<Self, Error<Ext2Error>> {
        let file = File::new(filesystem, inode_number)?;
        let entries = Mutex::new(Self::parse(&file)?);

        Ok(Self { file, entries })
    }

    /// Parse this inode's content as a list of directory entries.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Entry::parse`].
    fn parse(file: &File<Dev>) -> Result<Vec<Vec<Entry>>, Error<Ext2Error>> {
        let fs = file.filesystem.lock();

        let block_size = u64::from(fs.superblock().block_size());
        let data_size = file.inode.data_size();
        let data_blocks = 1 + (data_size - 1) / block_size;

        let mut indirected_blocks = file.inode.indirected_blocks(&fs)?;
        // SAFETY: there are at most u32::MAX blocks on this filesystem
        indirected_blocks.truncate_back_data_blocks(unsafe { u32::try_from(data_blocks).unwrap_unchecked() });

        let mut entries = Vec::new();

        for block in indirected_blocks.flatten_data_blocks() {
            let mut entries_in_block = Vec::new();
            let mut accumulated_size = 0_u64;
            while accumulated_size < block_size {
                let starting_addr = Address::from(
                    u64_to_usize(u64::from(block) * block_size + accumulated_size).map_err(Error::from_infallible)?,
                );

                let entry = Entry::parse(&fs, starting_addr)?;
                accumulated_size += u64::from(entry.rec_len);
                entries_in_block.push(entry);
            }
            entries.push(entries_in_block);
        }

        Ok(entries)
    }

    /// Updates the inner `entries` field of this directory.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Entry::parse`].
    fn update_inner_entries(&self) -> Result<(), Error<Ext2Error>> {
        *self.entries.lock() = Self::parse(&self.file)?;
        Ok(())
    }

    /// Writes all the entries of the block `block_index`.
    ///
    /// This function does not perform any check: the entries **MUST** be in a coherent state. It is recommanded to
    /// perform [`defragment`](Directory::defragment) beforehand.
    ///
    /// # Safety
    ///
    /// Must ensure that the entries are in a valid state regarding to the completion of data blocks and the number of
    /// entry per data block. Furthermore, `block_index` must be a valid index of `self.entries`.
    unsafe fn write_block_entry(&mut self, block_index: usize) -> Result<(), Error<Ext2Error>> {
        let block_size = u64::from(self.file.filesystem.lock().superblock().block_size());
        self.file.seek(SeekFrom::Start(usize_to_u64(block_index) * block_size))?;

        let mut buffer = Vec::new();
        let entries = self.entries.lock();
        for entry in unsafe { entries.get_unchecked(block_index) } {
            buffer.append(&mut entry.as_bytes().clone());
            buffer.append(&mut vec![0_u8; u32_to_usize(entry.free_space().into())]);
        }
        self.file.write_all(&buffer)?;

        Ok(())
    }

    /// Writes all the entries.
    ///
    /// This function does not perform any check: the entries **MUST** be in a coherent state. It is recommanded to
    /// perform [`defragment`](Directory::defragment) beforehand.
    ///
    /// # Safety
    ///
    /// Must ensure that the entries are in a valid state regarding to the completion of data blocks and the number of
    /// entry per data block.
    unsafe fn write_all_entries(&mut self) -> Result<(), Error<Ext2Error>> {
        self.file.truncate(0)?;
        let nb_entries = self.entries.lock().len();
        for block_index in 0..nb_entries {
            unsafe { self.write_block_entry(block_index) }?;
        }
        Ok(())
    }

    /// Defragments the directory by compacting (if necessary) all the entries.
    fn defragment(&self) {
        let block_size = u16::try_from(self.file.filesystem.lock().superblock().block_size())
            .expect("Ill-formed superblock: block size should be castable in a u16");

        let mut new_entries = Vec::new();

        let mut entries_in_block = Vec::<Entry>::new();
        let mut accumulated_size = 0_u16;
        for mut entry in self.entries.lock().clone().into_iter().flatten() {
            if accumulated_size + entry.minimal_size() > block_size {
                if let Some(ent) = entries_in_block.last_mut() {
                    ent.rec_len = block_size - accumulated_size;
                }
                new_entries.push(entries_in_block);
                accumulated_size = 0;
                entries_in_block = Vec::new();
            }
            entry.rec_len = entry.minimal_size();
            accumulated_size += entry.minimal_size();
            entries_in_block.push(entry);
        }

        if let Some(ent) = entries_in_block.last_mut() {
            ent.rec_len = block_size - accumulated_size;
            new_entries.push(entries_in_block);
        }

        *self.entries.lock() = new_entries;
    }

    /// Returns, if it exists, the block index and the entry index containing at least `necessary_space` free space.
    fn find_space(&self, necessary_space: u16) -> Option<(usize, usize)> {
        for (block_index, entries_in_block) in self.entries.lock().iter().enumerate() {
            for (entry_index, entry) in entries_in_block.iter().enumerate() {
                if entry.free_space() >= necessary_space {
                    return Some((block_index, entry_index));
                }
            }
        }

        None
    }
}

impl<Dev: Device> Clone for Directory<Dev> {
    fn clone(&self) -> Self {
        Self {
            file: self.file.clone(),
            entries: Mutex::new(self.entries.lock().clone()),
        }
    }
}

impl_file!(Directory);

impl<Dev: Device> file::DirectoryRead for Directory<Dev> {
    type BlockDevice = BlockDevice<Dev>;
    type CharacterDevice = CharacterDevice<Dev>;
    type Fifo = Fifo<Dev>;
    type Regular = Regular<Dev>;
    type Socket = Socket<Dev>;
    type SymbolicLink = SymbolicLink<Dev>;

    fn entries(&self) -> Result<Vec<file::DirectoryEntry<'_, Self>>, Error<Ext2Error>> {
        let mut entries = Vec::new();

        self.update_inner_entries()?;

        for entry in self.entries.lock().iter().flatten() {
            entries.push(DirectoryEntry {
                filename: entry
                    .name
                    .clone()
                    .try_into()
                    .unwrap_or_else(|_| panic!("The entry with name {:?} is not a valid UTF-8 sequence", entry.name)),
                file: self.file.filesystem.file(entry.inode)?,
            });
        }

        Ok(entries)
    }
}

impl<Dev: Device> file::Directory for Directory<Dev> {
    fn add_entry(
        &mut self,
        name: UnixStr<'_>,
        file_type: Type,
        permissions: Permissions,
        user_id: Uid,
        group_id: Gid,
    ) -> Result<TypeWithFile<Self>, Error<Self::FsError>> {
        if let Ok(file) = self.entry(name.clone())
            && file.is_some()
        {
            return Err(Error::Fs(FsError::EntryAlreadyExist(name.to_string())));
        }

        let mut fs = self.file.filesystem.lock();
        let block_size = fs.superblock().block_size();

        let inode_number = fs.free_inode()?;
        fs.allocate_inode(
            inode_number,
            TypePermissions::from(permissions) | TypePermissions::from(file_type),
            user_id
                .0
                .try_into()
                .map_err(|_| Error::Fs(FsError::Implementation(Ext2Error::UidTooLarge(user_id.0))))?,
            group_id
                .0
                .try_into()
                .map_err(|_| Error::Fs(FsError::Implementation(Ext2Error::GidTooLarge(group_id.0))))?,
            Flags::empty(),
            0,
            [0; 12],
        )?;

        let file_type_feature = fs.options.file_type;

        drop(fs);

        if file_type == Type::Directory {
            let mut dir = File::new(&self.file.filesystem, inode_number)?;
            let self_and_parent = [
                &Entry {
                    inode: inode_number,
                    rec_len: 9,
                    name_len: 1,
                    file_type: if file_type_feature { u8::from(FileType::Dir) } else { 0 },
                    // SAFETY: "." is a valid CString
                    name: unsafe { CString::from_vec_unchecked(vec![b'.']) },
                },
                &Entry {
                    inode: self.file.inode_number,
                    rec_len: u16::try_from(block_size - 9)
                        .expect("Ill-formed superblock: block size should be castable in a u16"),
                    name_len: 2,
                    file_type: if file_type_feature { u8::from(FileType::Dir) } else { 0 },
                    // SAFETY: ".." is a valid CString
                    name: unsafe { CString::from_vec_unchecked(vec![b'.', b'.']) },
                },
            ];

            let self_and_parent_bytes = self_and_parent.map(Entry::as_bytes).concat();
            dir.seek(SeekFrom::Start(0))?;
            dir.write_all(&self_and_parent_bytes)?;
            dir.flush()?;
        }

        let mut new_entry = Entry {
            inode: inode_number,
            rec_len: 0,
            name_len: u8::try_from(name.to_string().len())
                .map_err(|_err| Error::Fs(FsError::Implementation(Ext2Error::NameTooLong(name.to_string()))))?,
            file_type: directory::FileType::from(file_type).into(),
            name: name.into(),
        };
        new_entry.rec_len = new_entry.minimal_size();
        if let Some((block_index, entry_index)) = self.find_space(new_entry.minimal_size()) {
            let mut self_entries = self.entries.lock();
            // SAFETY: `find_space` returns a valid block index
            let entries_in_block = unsafe { self_entries.get_unchecked_mut(block_index) };
            // SAFETY: `find_space` returs a valid entry index
            let previous_entry = unsafe { entries_in_block.get_unchecked_mut(entry_index) };

            new_entry.rec_len = previous_entry.rec_len - previous_entry.minimal_size();
            previous_entry.rec_len = previous_entry.minimal_size();

            entries_in_block.insert(entry_index + 1, new_entry);
            drop(self_entries);

            // SAFETY: all necessary changes have been made
            unsafe { self.write_block_entry(block_index) }?;
        } else {
            self.entries.lock().push(vec![new_entry]);
            self.defragment();
            // SAFETY: `defragment` has been called above
            unsafe { self.write_all_entries() }?;
        }

        let fs = self.file.filesystem.lock();
        let mut new_inode = fs.inode(inode_number)?;

        let time = fs.get_time();
        new_inode.atime = time;
        new_inode.mtime = time;
        new_inode.ctime = time;

        unsafe { Inode::write_on_device(&fs, inode_number, new_inode)? };
        drop(fs);

        self.file.filesystem.file(inode_number)
    }

    fn remove_entry(&mut self, entry_name: crate::path::UnixStr) -> Result<(), Error<Self::FsError>> {
        if entry_name == *CUR_DIR || entry_name == *PARENT_DIR {
            return Err(Error::Fs(FsError::RemoveRefused));
        }

        let block_size = u64::from(self.file.filesystem.lock().superblock().block_size());
        let entry_name_cstring = Into::<CString>::into(entry_name.clone());
        let entries_clone = self.entries.lock().clone();
        for (block_index, entries_in_block) in entries_clone.into_iter().enumerate() {
            for (index, entry) in entries_in_block.clone().into_iter().enumerate() {
                if entry.name == entry_name_cstring {
                    let mut self_entries = self.entries.lock();
                    // SAFETY: `block_index` is returned by `enumerate`
                    let block = unsafe { self_entries.get_unchecked_mut(block_index) };
                    block.remove(index);

                    // Case: the removed entry is not the first of the block
                    if index > 0
                        && let Some(previous_entry) = block.get_mut(index - 1)
                    {
                        previous_entry.rec_len += entry.rec_len;
                    }
                    // Case: the removed entry is the first of the block
                    else if let Some(next_entry) = block.get_mut(index) {
                        next_entry.rec_len += entry.rec_len;
                    }
                    // Case: the removed entry is the first and only entry of the block
                    else {
                        self.entries.lock().remove(block_index);
                        let nb_entries = self.entries.lock().len();
                        self.file.truncate(block_size * usize_to_u64(nb_entries))?;
                    }

                    let block_len = block.len();
                    drop(self_entries);
                    if index > 0 || block_len > 0 {
                        // SAFETY: the content of this block is coherent
                        unsafe { self.write_block_entry(block_index) }?;
                    } else {
                        let nb_entries = self.entries.lock().len();
                        for i in block_index..nb_entries {
                            // SAFETY: this block is untouched
                            unsafe { self.write_block_entry(i) }?;
                        }
                    }

                    if let TypeWithFile::Directory(mut dir) = self.file.filesystem.file(entry.inode)? {
                        let mut new_inode = self.file.inode;
                        new_inode.links_count -= 1;
                        let sub_entries = dir.entries.lock().clone().into_iter().flatten().collect_vec();
                        for sub_entry in sub_entries {
                            let sub_entry_name: UnixStr<'_> = sub_entry.name.clone().try_into().unwrap_or_else(|_| {
                                panic!("The entry with name {:?} is not a valid UTF-8 sequence", sub_entry.name)
                            });
                            if sub_entry_name != *CUR_DIR && sub_entry_name != *PARENT_DIR {
                                dir.remove_entry(sub_entry_name.clone())?;
                            }
                        }

                        // SAFETY: the new number of links is exactly the previous one minus all the children that are
                        // directories
                        unsafe {
                            self.file.set_inode(&new_inode)?;
                        };
                    }

                    let mut fs = self.file.filesystem.lock();
                    return fs.deallocate_inode(entry.inode);
                }
            }
        }

        Err(Error::Fs(FsError::NotFound(entry_name.to_string())))
    }
}

/// Interface for ext2's symbolic links.
#[derive(Debug)]
pub struct SymbolicLink<Dev: Device> {
    /// Inner file containing the generic file.
    file: File<Dev>,

    /// Read/Write offset (can be manipulated with [`Seek`]).
    pointed_file: String,
}

impl<Dev: Device> SymbolicLink<Dev> {
    /// Returns a new ext2's [`SymbolicLink`] from an [`Ext2Fs`] instance and the inode number of the file.
    ///
    /// # Errors
    ///
    /// Returns a [`BadString`](Ext2Error::BadString) if the content of the given inode does not look like a valid path.
    ///
    /// Returns a [`NameTooLong`](crate::fs::error::FsError::NameTooLong) if the size of the inode's content is greater
    /// than [`PATH_MAX`].
    ///
    /// Otherwise, returns the same errors as [`Ext2::inode`](super::Ext2::inode).
    pub fn new(filesystem: &Ext2Fs<Dev>, inode_number: u32) -> Result<Self, Error<Ext2Error>> {
        let file = File::new(&filesystem.clone(), inode_number)?;

        let data_size = usize::try_from(file.inode.data_size()).unwrap_or(PATH_MAX);

        let mut buffer = vec![0_u8; data_size];

        if data_size < SYMBOLIC_LINK_INODE_STORE_LIMIT {
            // SAFETY: it is always possible to read a slice of u8
            buffer.clone_from_slice(unsafe {
                core::slice::from_raw_parts(addr_of!(file.inode.direct_block_pointers).cast(), data_size)
            });
        } else {
            let _: usize = file.inode.read_data(&filesystem.lock(), &mut buffer, 0)?;
        }
        let pointed_file = buffer
            .split(|char| *char == b'\0')
            .next()
            .ok_or(Ext2Error::BadString)
            .map_err(FsError::Implementation)?
            .to_vec();
        Ok(Self {
            file,
            pointed_file: String::from_utf8(pointed_file)
                .map_err(|_err| Ext2Error::BadString)
                .map_err(FsError::Implementation)?,
        })
    }
}

impl<Dev: Device> Clone for SymbolicLink<Dev> {
    fn clone(&self) -> Self {
        Self {
            file: self.file.clone(),
            pointed_file: self.pointed_file.clone(),
        }
    }
}

impl_file!(SymbolicLink);

impl<Dev: Device> file::SymbolicLinkRead for SymbolicLink<Dev> {
    fn get_pointed_file(&self) -> Result<&str, Error<Self::FsError>> {
        Ok(&self.pointed_file)
    }
}

impl<Dev: Device> file::SymbolicLink for SymbolicLink<Dev> {
    fn set_pointed_file(&mut self, pointed_file: &str) -> Result<(), Error<Self::FsError>> {
        let bytes = pointed_file.as_bytes();

        if bytes.len() > PATH_MAX {
            return Err(Error::Fs(FsError::NameTooLong(pointed_file.to_owned())));
        } else if bytes.len() > SYMBOLIC_LINK_INODE_STORE_LIMIT {
            if self.pointed_file.len() <= SYMBOLIC_LINK_INODE_STORE_LIMIT {
                let mut new_inode = self.file.inode;

                let data_ptr = addr_of_mut!(new_inode.direct_block_pointers).cast::<u8>();
                // SAFETY: there are `SYMBOLIC_LINK_INODE_STORE_LIMIT` bytes available to store the data
                let data_slice = unsafe { core::slice::from_raw_parts_mut(data_ptr, SYMBOLIC_LINK_INODE_STORE_LIMIT) };
                data_slice.clone_from_slice(&[b'\0'; SYMBOLIC_LINK_INODE_STORE_LIMIT]);

                let fs = self.file.filesystem.lock();
                // SAFETY: the starting address correspond to the one of this inode
                unsafe {
                    Inode::write_on_device(&fs, self.file.inode_number, new_inode)?;
                };

                drop(fs);

                self.file.update_inner_inode()?;
            }

            self.file.seek(SeekFrom::Start(0))?;
            self.file.write_all(bytes)?;
            self.file.truncate(usize_to_u64(bytes.len()))?;
        } else {
            self.file.truncate(0)?;

            let mut new_inode = self.file.inode;
            // SAFETY: `bytes.len() < PATH_MAX << u32::MAX`
            new_inode.size = unsafe { u32::try_from(bytes.len()).unwrap_unchecked() };

            let data_ptr = addr_of_mut!(new_inode.direct_block_pointers).cast::<u8>();
            // SAFETY: there are `SYMBOLIC_LINK_INODE_STORE_LIMIT` bytes available to store the data
            let data_slice = unsafe { core::slice::from_raw_parts_mut(data_ptr, bytes.len()) };
            data_slice.clone_from_slice(bytes);

            let fs = self.file.filesystem.lock();
            // SAFETY: the starting address correspond to the one of this inode
            unsafe {
                Inode::write_on_device(&fs, self.file.inode_number, new_inode)?;
            };
            drop(fs);

            self.file.update_inner_inode()?;
        }

        pointed_file.clone_into(&mut self.pointed_file);
        Ok(())
    }
}

macro_rules! generic_file {
    ($id:ident) => {
        #[doc = concat!("Basic implementation of a [`", stringify!($id), "`](crate::fs::file::", stringify!($id), ") for ext2.")]
        #[derive(Debug)]
        pub struct $id<Dev: Device> {
            /// Inner file containing the generic file.
            file: File<Dev>,
        }

        impl<Dev: Device> Clone for $id<Dev> {
            fn clone(&self) -> Self {
                Self {
                    file: self.file.clone(),
                }
            }
        }

        impl_file!($id);

        impl<Dev: Device> $id<Dev> {
            #[doc = concat!("Returns a new ext2's [`", stringify!($id), "`] from an [`Ext2Fs`] instance and the inode number of the file.")]
            ///
            /// # Errors
            ///
            /// Returns the same errors as [`File::new`].
            pub fn new(filesystem: &Ext2Fs<Dev>, inode_number: u32) -> Result<Self, Error<Ext2Error>> {
                Ok(Self { file: File::new(filesystem, inode_number)? })
            }
        }

        impl<Dev: Device> crate::fs::file::$id for $id<Dev> {}
        impl<Dev: Device> crate::fs::file::${concat($id, Read)}  for $id<Dev> {}
    };
}

generic_file!(Fifo);
generic_file!(CharacterDevice);
generic_file!(BlockDevice);
generic_file!(Socket);

#[cfg(test)]
mod test {
    use alloc::string::{String, ToString};
    use alloc::vec;
    use alloc::vec::Vec;
    use core::time::Duration;
    use std::fs::File;

    use deku::no_std_io::{Read, Seek, SeekFrom, Write};
    use itertools::Itertools;

    use crate::arch::usize_to_u64;
    use crate::dev::address::Address;
    use crate::fs::FilesystemRead;
    use crate::fs::ext2::directory::Entry;
    use crate::fs::ext2::file::Directory;
    use crate::fs::ext2::inode::{Inode, ROOT_DIRECTORY_INODE, TypePermissions};
    use crate::fs::ext2::{Ext2, Ext2Fs};
    use crate::fs::file::{DirectoryRead, FileRead, Regular, SymbolicLink, SymbolicLinkRead, Type, TypeWithFile};
    use crate::fs::permissions::Permissions;
    use crate::fs::types::{Gid, Mode, Uid};
    use crate::path::{Path, UnixStr};
    use crate::tests::{LOREM, LOREM_LENGTH, new_device_id};

    fn parse_root(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let root = Directory::new(&ext2, ROOT_DIRECTORY_INODE).unwrap();
        assert_eq!(
            root.entries
                .lock()
                .clone()
                .into_iter()
                .flatten()
                .map(|entry| entry.name.to_string_lossy().to_string())
                .collect::<Vec<String>>(),
            vec![".", "..", "lost+found", "big_file", "symlink"]
        );
    }

    fn parse_root_entries(file: File) {
        let fs = Ext2::new(file, new_device_id()).unwrap();
        let root_inode = Inode::parse(&fs, ROOT_DIRECTORY_INODE).unwrap();

        let dot = Entry::parse(
            &fs,
            Address::new(u64::from(root_inode.direct_block_pointers[0]) * u64::from(fs.superblock().block_size())),
        )
        .unwrap();
        let two_dots = Entry::parse(
            &fs,
            Address::new(
                u64::from(root_inode.direct_block_pointers[0]) * u64::from(fs.superblock().block_size())
                    + u64::from(dot.rec_len),
            ),
        )
        .unwrap();
        let lost_and_found = Entry::parse(
            &fs,
            Address::new(
                (u64::from(root_inode.direct_block_pointers[0]) * u64::from(fs.superblock().block_size()))
                    + u64::from(dot.rec_len + two_dots.rec_len),
            ),
        )
        .unwrap();
        let big_file = Entry::parse(
            &fs,
            Address::new(
                (u64::from(root_inode.direct_block_pointers[0]) * u64::from(fs.superblock().block_size()))
                    + u64::from(dot.rec_len + two_dots.rec_len + lost_and_found.rec_len),
            ),
        )
        .unwrap();
        let symlink = Entry::parse(
            &fs,
            Address::new(
                (u64::from(root_inode.direct_block_pointers[0]) * u64::from(fs.superblock().block_size()))
                    + u64::from(dot.rec_len + two_dots.rec_len + lost_and_found.rec_len + big_file.rec_len),
            ),
        )
        .unwrap();

        assert_eq!(dot.name.as_c_str().to_string_lossy(), ".");
        assert_eq!(two_dots.name.as_c_str().to_string_lossy(), "..");
        assert_eq!(lost_and_found.name.as_c_str().to_string_lossy(), "lost+found");
        assert_eq!(big_file.name.as_c_str().to_string_lossy(), "big_file");
        assert_eq!(symlink.name.as_c_str().to_string_lossy(), "symlink");
    }

    fn parse_big_file_inode_data(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let root = Directory::new(&ext2, ROOT_DIRECTORY_INODE).unwrap();

        let fs = ext2.lock();
        let big_file_inode_number = root
            .entries
            .lock()
            .iter()
            .flatten()
            .find(|entry| entry.name.to_string_lossy() == "big_file")
            .unwrap()
            .inode;
        let big_file_inode = fs.inode(big_file_inode_number).unwrap();

        let singly_indirect_block_pointer = big_file_inode.singly_indirect_block_pointer;
        let doubly_indirect_block_pointer = big_file_inode.doubly_indirect_block_pointer;
        assert_ne!(singly_indirect_block_pointer, 0);
        assert_ne!(doubly_indirect_block_pointer, 0);

        assert_ne!(big_file_inode.data_size(), 0);

        for offset in 0_usize..1_024_usize {
            let mut buffer = [0_u8; 1_024];
            big_file_inode.read_data(&fs, &mut buffer, usize_to_u64(offset * 1_024)).unwrap();

            assert_eq!(buffer.iter().all_equal_value(), Ok(&1));
        }

        let mut buffer = [0_u8; 1_024];
        big_file_inode.read_data(&fs, &mut buffer, 0x0010_0000).unwrap();
        assert_eq!(buffer.iter().all_equal_value(), Ok(&0));
    }

    fn read_file(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();

        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut foo) = root.entry(UnixStr::new("foo.txt").unwrap()).unwrap().unwrap() else {
            panic!("`foo.txt` has been created as a regular file")
        };

        assert_eq!(foo.file.read_all().unwrap(), b"Hello world!\n");
    }

    fn read_file_with_offset(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();

        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut foo) = root.entry(UnixStr::new("foo.txt").unwrap()).unwrap().unwrap() else {
            panic!("`foo.txt` has been created as a regular file")
        };

        foo.seek(SeekFrom::Start(6)).unwrap();
        let mut buf = [0; 7];
        foo.file.read_exact(&mut buf).unwrap();

        assert_eq!(&buf, b"world!\n");
    }

    fn read_lorem(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();

        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut lorem) = root.entry(UnixStr::new("lorem.txt").unwrap()).unwrap().unwrap() else {
            panic!("`lorem.txt` has been created as a regular file")
        };

        assert_eq!(LOREM.as_bytes(), lorem.read_all().unwrap());

        lorem.seek(SeekFrom::Start(504)).unwrap();
        let mut buf = [0_u8; 3000];
        lorem.read_exact(&mut buf).unwrap();
        assert_eq!(buf, LOREM.as_bytes()[504..(504 + 3000)]);

        lorem.seek(SeekFrom::End(-830)).unwrap();
        let mut buf = [0_u8; 300];
        lorem.read_exact(&mut buf).unwrap();
        assert_eq!(buf, LOREM.as_bytes()[(LOREM_LENGTH - 830)..(LOREM_LENGTH - 830 + 300)]);
    }

    fn read_symlink(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let root = Directory::new(&ext2, ROOT_DIRECTORY_INODE).unwrap();

        let TypeWithFile::SymbolicLink(symlink) = root.entry(UnixStr::new("symlink").unwrap()).unwrap().unwrap() else {
            panic!("`symlink` has been created as a symbolic link")
        };

        assert_eq!(symlink.pointed_file, "big_file");
    }

    fn set_inode(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut foo) = root.entry(UnixStr::new("foo.txt").unwrap()).unwrap().unwrap() else {
            panic!("`foo.txt` has been created as a regular file")
        };

        let mut new_inode = foo.file.inode;
        new_inode.uid = 0x1234;
        new_inode.gid = 0x2345;
        new_inode.flags = 0xabcd;
        unsafe { foo.file.set_inode(&new_inode) }.unwrap();

        assert_eq!(foo.file.inode, Inode::parse(&ext2.lock(), foo.file.inode_number).unwrap());
        assert_eq!(foo.file.inode, new_inode);
    }

    fn write_file_dbp_replace_without_allocation(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut foo) = root.entry(UnixStr::new("foo.txt").unwrap()).unwrap().unwrap() else {
            panic!("`foo.txt` has been created as a regular file")
        };

        foo.seek(SeekFrom::Start(6)).unwrap();
        let replace_text = b"earth";
        foo.write_all(replace_text).unwrap();
        foo.flush().unwrap();

        assert_eq!(foo.file.inode, Inode::parse(&ext2.lock(), foo.file.inode_number).unwrap());

        assert_eq!(String::from_utf8(foo.read_all().unwrap()).unwrap(), "Hello earth!\n");
    }

    fn write_file_dbp_extend_without_allocation(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut foo) = root.entry(UnixStr::new("foo.txt").unwrap()).unwrap().unwrap() else {
            panic!("`foo.txt` has been created as a regular file")
        };

        foo.seek(SeekFrom::Start(6)).unwrap();
        let replace_text = b"earth!\nI love dogs!\n";
        foo.write_all(replace_text).unwrap();
        foo.flush().unwrap();

        assert_eq!(foo.file.inode, Inode::parse(&ext2.lock(), foo.file.inode_number).unwrap());

        assert_eq!(foo.read_all().unwrap(), b"Hello earth!\nI love dogs!\n");
    }

    fn write_file_dbp_extend_with_allocation(file: File) {
        const BYTES_TO_WRITE: usize = 12_000;

        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut foo) = root.entry(UnixStr::new("foo.txt").unwrap()).unwrap().unwrap() else {
            panic!("`foo.txt` has been created as a regular file")
        };

        let replace_text = &[b'a'; BYTES_TO_WRITE];
        foo.write_all(replace_text).unwrap();
        foo.flush().unwrap();

        assert_eq!(foo.file.inode, Inode::parse(&ext2.lock(), foo.file.inode_number).unwrap());

        assert_eq!(foo.read_all().unwrap().len(), BYTES_TO_WRITE);
        assert_eq!(foo.read_all().unwrap().into_iter().all_equal_value(), Ok(b'a'));
    }

    fn write_file_singly_indirect_block_pointer(file: File) {
        const BYTES_TO_WRITE: usize = 23_000;

        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut foo) = root.entry(UnixStr::new("foo.txt").unwrap()).unwrap().unwrap() else {
            panic!("`foo.txt` has been created as a regular file")
        };

        let mut replace_text = vec![];
        for i in 0..=u8::MAX {
            replace_text.append(&mut vec![i; BYTES_TO_WRITE / 256]);
        }
        replace_text.append(&mut vec![b'a'; BYTES_TO_WRITE - 256 * (BYTES_TO_WRITE / 256)]);

        foo.write_all(&replace_text).unwrap();
        foo.flush().unwrap();

        assert_eq!(foo.file.inode, Inode::parse(&ext2.lock(), foo.file.inode_number).unwrap());
        assert_eq!(foo.read_all().unwrap().len(), BYTES_TO_WRITE);
        assert_eq!(foo.read_all().unwrap(), replace_text);
    }

    fn write_file_doubly_indirect_block_pointer(file: File) {
        const BYTES_TO_WRITE: usize = 400_000;

        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut foo) = root.entry(UnixStr::new("foo.txt").unwrap()).unwrap().unwrap() else {
            panic!("`foo.txt` has been created as a regular file")
        };

        let mut replace_text = vec![];
        for i in 0..=u8::MAX {
            replace_text.append(&mut vec![i; BYTES_TO_WRITE / 256]);
        }
        replace_text.append(&mut vec![b'a'; BYTES_TO_WRITE - 256 * (BYTES_TO_WRITE / 256)]);

        foo.write_all(&replace_text).unwrap();
        foo.flush().unwrap();

        assert_eq!(foo.file.inode, Inode::parse(&ext2.lock(), foo.file.inode_number).unwrap());
        assert_eq!(foo.read_all().unwrap().len(), BYTES_TO_WRITE);
        assert_eq!(foo.read_all().unwrap(), replace_text);
    }

    fn write_file_triply_indirect_block_pointer(file: File) {
        const BYTES_TO_WRITE: usize = 70_000_000;

        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut foo) = root.entry(UnixStr::new("foo.txt").unwrap()).unwrap().unwrap() else {
            panic!("`foo.txt` has been created as a regular file")
        };

        let mut replace_text = vec![b'a'; BYTES_TO_WRITE / 2];
        replace_text.append(&mut vec![b'b'; BYTES_TO_WRITE / 2]);
        foo.write_all(&replace_text).unwrap();
        foo.flush().unwrap();

        assert_eq!(foo.file.inode, Inode::parse(&ext2.lock(), foo.file.inode_number).unwrap());
        assert_eq!(foo.read_all().unwrap().len(), BYTES_TO_WRITE);
        assert_eq!(foo.read_all().unwrap(), replace_text);
    }

    fn write_file_twice(file: File) {
        const BYTES_TO_WRITE: usize = 23_000;

        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut foo) = root.entry(UnixStr::new("foo.txt").unwrap()).unwrap().unwrap() else {
            panic!("`foo.txt` has been created as a regular file")
        };

        let mut replace_text = vec![];
        for i in 0..=u8::MAX {
            replace_text.append(&mut vec![i; BYTES_TO_WRITE / 256]);
        }
        replace_text.append(&mut vec![b'a'; BYTES_TO_WRITE - 256 * (BYTES_TO_WRITE / 256)]);

        foo.write_all(&replace_text[..BYTES_TO_WRITE / 2]).unwrap();
        foo.write_all(&replace_text[BYTES_TO_WRITE / 2..]).unwrap();
        foo.flush().unwrap();

        assert_eq!(foo.file.inode, Inode::parse(&ext2.lock(), foo.file.inode_number).unwrap());
        assert_eq!(foo.read_all().unwrap().len(), BYTES_TO_WRITE);
        assert_eq!(foo.read_all().unwrap(), replace_text);
    }

    #[allow(clippy::similar_names)]
    fn file_mode(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut foo) = root.entry(UnixStr::new("foo.txt").unwrap()).unwrap().unwrap() else {
            panic!("`foo.txt` has been created as a regular file")
        };

        assert_eq!(foo.get_type(), Type::Regular);
        assert_eq!(
            foo.permissions(),
            Permissions::USER_READ | Permissions::USER_WRITE | Permissions::GROUP_READ | Permissions::OTHER_READ
        );

        crate::fs::file::File::set_mode(&mut foo, Mode::from(Permissions::USER_READ | Permissions::USER_WRITE))
            .unwrap();
        crate::fs::file::File::set_uid(&mut foo, Uid(1)).unwrap();
        crate::fs::file::File::set_gid(&mut foo, Gid(2)).unwrap();

        let fs = ext2.lock();
        let inode = Inode::parse(&fs, foo.file.inode_number).unwrap();

        let mode = inode.mode;
        assert_eq!(mode, (TypePermissions::REGULAR_FILE | TypePermissions::USER_R | TypePermissions::USER_W).bits());
        let uid = inode.uid;
        assert_eq!(uid, 1);
        let gid = inode.gid;
        assert_eq!(gid, 2);
    }

    fn file_truncation(file: File) {
        const BYTES_TO_WRITE: usize = 400_000;

        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut foo) = root.entry(UnixStr::new("foo.txt").unwrap()).unwrap().unwrap() else {
            panic!("`foo.txt` has been created as a regular file")
        };

        let initial_free_block_number = ext2.lock().superblock().base().free_blocks_count;
        let initial_foo_size = foo.file.inode.data_size();

        let replace_text = vec![b'a'; BYTES_TO_WRITE];
        foo.write_all(&replace_text).unwrap();
        foo.flush().unwrap();

        assert_eq!(foo.file.inode.data_size(), usize_to_u64(BYTES_TO_WRITE));

        foo.truncate(10000).unwrap();

        assert_eq!(foo.file.inode.data_size(), 10000);
        assert_eq!(foo.read_all().unwrap().len(), 10000);

        foo.truncate(initial_foo_size).unwrap();
        let new_free_block_number = ext2.lock().superblock().base().free_blocks_count;

        assert_eq!(foo.file.inode.data_size(), initial_foo_size);
        assert!(new_free_block_number >= initial_free_block_number); // Non used blocks could be deallocated
    }

    fn file_symlinks(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.");
        };
        let TypeWithFile::SymbolicLink(mut bar) = root.entry(UnixStr::new("bar.txt").unwrap()).unwrap().unwrap() else {
            panic!("`bar.txt` has been created as a symbolic link");
        };

        assert_eq!(bar.get_type(), Type::SymbolicLink);

        assert_eq!(bar.get_pointed_file().unwrap(), "foo.txt");
        assert_eq!(bar.file.inode.data_size(), 7);

        bar.set_pointed_file("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
            .unwrap();
        assert_eq!(
            bar.get_pointed_file().unwrap(),
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
        );
        assert_eq!(
            bar.file.read_all().unwrap(),
            b"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
        );
        assert_eq!(bar.file.inode.data_size(), 70);

        bar.set_pointed_file("foo.txt").unwrap();

        assert_eq!(bar.get_pointed_file().unwrap(), "foo.txt");
        assert!(bar.file.read_all().is_err());
        assert_eq!(bar.file.inode.data_size(), 7);
    }

    fn new_files(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(mut root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.");
        };

        assert!(root.entry(UnixStr::new("boo.txt").unwrap()).is_ok_and(|res| res.is_none()));
        let TypeWithFile::Regular(mut boo) = crate::fs::file::Directory::add_entry(
            &mut root,
            UnixStr::new("boo.txt").unwrap(),
            Type::Regular,
            Permissions::USER_READ | Permissions::USER_WRITE | Permissions::USER_EXECUTION,
            Uid(0),
            Gid(0),
        )
        .unwrap() else {
            panic!("boo has been created as a regular file.")
        };

        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.");
        };
        assert!(root.entry(UnixStr::new("boo.txt").unwrap()).is_ok_and(|res| res.is_some()));

        let ctime = boo.file.inode.ctime;
        let atime = boo.file.inode.atime;
        let mtime = boo.file.inode.mtime;
        assert_ne!(ctime, 0);
        assert_ne!(atime, 0);
        assert_ne!(mtime, 0);

        let root_atime = root.file.inode.atime;
        assert!(atime - root_atime < 1);

        boo.write_all(b"Hello earth!\n").unwrap();
        assert_eq!(boo.read_all().unwrap(), b"Hello earth!\n");
    }

    #[allow(clippy::similar_names)]
    fn remove_files(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(mut root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.");
        };

        assert!(root.entry(UnixStr::new("bar.txt").unwrap()).is_ok_and(|res| res.is_some()));
        crate::fs::file::Directory::remove_entry(&mut root, UnixStr::new("bar.txt").unwrap()).unwrap();
        let TypeWithFile::Directory(mut root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.");
        };
        assert!(root.entry(UnixStr::new("bar.txt").unwrap()).is_ok_and(|res| res.is_none()));

        let TypeWithFile::Regular(ex1) = ext2
            .get_file(&Path::new(UnixStr::new("/folder/ex1.txt").unwrap()), root.clone(), false)
            .unwrap()
        else {
            panic!("ex1.txt is a regular file.");
        };
        let ex1_inode = ex1.file.inode_number;

        let TypeWithFile::SymbolicLink(ex2) = ext2
            .get_file(&Path::new(UnixStr::new("/folder/ex2.txt").unwrap()), root.clone(), false)
            .unwrap()
        else {
            panic!("ex2.txt is a symbolic link.");
        };
        let ex2_inode = ex2.file.inode_number;

        let fs = ext2.lock();
        let superblock = fs.superblock();
        let ex1_bitmap = fs.get_inode_bitmap(Inode::block_group(superblock, ex1_inode)).unwrap();
        assert!(Inode::is_used(ex1_inode, superblock, &ex1_bitmap));
        let ex2_bitmap = fs.get_inode_bitmap(Inode::block_group(superblock, ex2_inode)).unwrap();
        assert!(Inode::is_used(ex2_inode, superblock, &ex2_bitmap));
        drop(fs);

        crate::fs::file::Directory::remove_entry(&mut root, UnixStr::new("folder").unwrap()).unwrap();

        let fs = ext2.lock();
        let superblock = fs.superblock();
        let ex1_bitmap = fs.get_inode_bitmap(Inode::block_group(superblock, ex1_inode)).unwrap();
        assert!(Inode::is_free(ex1_inode, superblock, &ex1_bitmap));
        let ex2_bitmap = fs.get_inode_bitmap(Inode::block_group(superblock, ex2_inode)).unwrap();
        assert!(Inode::is_free(ex2_inode, superblock, &ex2_bitmap));
    }

    fn atime_and_mtime(file: File) {
        let ext2 = Ext2Fs::new(file, new_device_id()).unwrap();
        let TypeWithFile::Directory(root) = ext2.file(ROOT_DIRECTORY_INODE).unwrap() else {
            panic!("The root is always a directory.")
        };
        let TypeWithFile::Regular(mut foo) = root.entry(UnixStr::new("foo.txt").unwrap()).unwrap().unwrap() else {
            panic!("`foo.txt` has been created as a regular file")
        };

        let atime = foo.file.inode.atime;
        let mtime = foo.file.inode.mtime;

        std::thread::sleep(Duration::from_secs(2));

        foo.write_all(b"Hello earth!").unwrap();

        let fs = ext2.lock();
        let new_inode = Inode::parse(&fs, foo.file.inode_number).unwrap();
        drop(fs);
        assert!(new_inode.atime > atime);
        assert!(new_inode.mtime > mtime);

        let atime = new_inode.atime;
        let mtime = new_inode.mtime;

        std::thread::sleep(Duration::from_secs(1));

        foo.write_all(b" and other planets too!").unwrap();

        let fs = ext2.lock();
        let new_inode = Inode::parse(&fs, foo.file.inode_number).unwrap();
        drop(fs);
        assert!(new_inode.atime < atime + 3);
        assert!(new_inode.mtime < mtime + 3);
    }

    mod generated {
        use crate::tests::{PostCheck, generate_fs_test};

        generate_fs_test!(parse_root, "./tests/fs/ext2/extended.ext2", PostCheck::Ext);
        generate_fs_test!(parse_root_entries, "./tests/fs/ext2/extended.ext2", PostCheck::Ext);
        generate_fs_test!(parse_big_file_inode_data, "./tests/fs/ext2/extended.ext2", PostCheck::Ext);
        generate_fs_test!(read_file, "./tests/fs/ext2/io_operations.ext2", PostCheck::Ext);
        generate_fs_test!(read_file_with_offset, "./tests/fs/ext2/io_operations.ext2", PostCheck::Ext);
        generate_fs_test!(read_lorem, "./tests/fs/ext2/io_operations.ext2", PostCheck::Ext);
        generate_fs_test!(read_symlink, "./tests/fs/ext2/extended.ext2", PostCheck::Ext);
        generate_fs_test!(
            write_file_dbp_extend_without_allocation,
            "./tests/fs/ext2/io_operations.ext2",
            PostCheck::Ext
        );
        generate_fs_test!(
            write_file_dbp_replace_without_allocation,
            "./tests/fs/ext2/io_operations.ext2",
            PostCheck::Ext
        );
        generate_fs_test!(write_file_dbp_extend_with_allocation, "./tests/fs/ext2/io_operations.ext2", PostCheck::Ext);
        generate_fs_test!(
            write_file_singly_indirect_block_pointer,
            "./tests/fs/ext2/io_operations.ext2",
            PostCheck::Ext
        );
        generate_fs_test!(
            write_file_doubly_indirect_block_pointer,
            "./tests/fs/ext2/io_operations.ext2",
            PostCheck::Ext
        );
        generate_fs_test!(
            write_file_triply_indirect_block_pointer,
            "./tests/fs/ext2/io_operations.ext2",
            PostCheck::Ext
        );
        generate_fs_test!(write_file_twice, "./tests/fs/ext2/io_operations.ext2", PostCheck::Ext);
        generate_fs_test!(file_mode, "./tests/fs/ext2/io_operations.ext2", PostCheck::Ext);
        generate_fs_test!(file_truncation, "./tests/fs/ext2/io_operations.ext2", PostCheck::Ext);
        generate_fs_test!(file_symlinks, "./tests/fs/ext2/io_operations.ext2", PostCheck::Ext);
        generate_fs_test!(new_files, "./tests/fs/ext2/io_operations.ext2", PostCheck::Ext);
        generate_fs_test!(remove_files, "./tests/fs/ext2/io_operations.ext2", PostCheck::Ext);
        generate_fs_test!(atime_and_mtime, "./tests/fs/ext2/io_operations.ext2", PostCheck::Ext);

        // Unsound changes on the ext2 filesystem are made so there should not be a e2fsck check afterward.
        generate_fs_test!(set_inode, "./tests/fs/ext2/io_operations.ext2", PostCheck::None);
    }
}
