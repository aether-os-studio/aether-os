//! General interface for Unix files.
//!
//! See [this Wikipedia page](https://en.wikipedia.org/wiki/Unix_file_types) and [the POSIX header of `<sys/stat.h>`](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_stat.h.html) for more information.

use alloc::vec::Vec;

use deku::no_std_io::{Read, Seek, Write};

use crate::error::Error;
use crate::fs::permissions::Permissions;
use crate::fs::types::{Blkcnt, Blksize, Dev, Gid, Ino, Mode, Nlink, Off, Timespec, Uid};
use crate::path::{PARENT_DIR, UnixStr};

/// Minimal stat structure.
///
/// More information on [the POSIX definition](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/sys_stat.h.html).
#[derive(Debug, Clone)]
pub struct Stat {
    /// Device ID of device containing file.
    pub dev: Dev,

    /// File serial number.
    pub ino: Ino,

    /// Mode of file.
    pub mode: Mode,

    /// Number of hard links to the file.
    pub nlink: Nlink,

    /// User ID of file.
    pub uid: Uid,

    /// Group ID of file.
    pub gid: Gid,

    /// Device ID (if file is character or block special).
    pub rdev: Dev,

    /// For regular files, the file size in bytes.
    ///
    /// For symbolic links, the length in bytes of the pathname contained in the symbolic link.
    pub size: Off,

    /// Last data access time stamp.
    pub atim: Timespec,

    /// Last data modification time stamp.
    pub mtim: Timespec,

    /// Last file status change time stamp.
    pub ctim: Timespec,

    /// A file system-specific preferred I/O block size for this object. In some file system types, this may vary from
    /// file to file.
    pub blksize: Blksize,

    /// Number of blocks allocated for this object.
    pub blkcnt: Blkcnt,
}

/// Base trait to ensure a common filesystem error type.
pub trait Base {
    /// Error type corresponding to the [`FileSystem`](crate::fs::Filesystem) implemented.
    type FsError: core::error::Error;
}

/// A readable UNIX file.
///
/// This type can be used alone for read-only filesystems.
pub trait FileRead: Base {
    /// Retrieves information about this file.
    fn stat(&self) -> Stat;

    /// Retrieves the [`Type`] of this file.
    fn get_type(&self) -> Type;

    /// Returns the [`Permissions`] of this file.
    fn permissions(&self) -> Permissions {
        self.stat().mode.into()
    }
}

/// Main trait for all UNIX files.
///
/// Defined in [this POSIX definition](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/V1_chap03.html#tag_03_164).
pub trait File: FileRead {
    /// Sets the [`Mode`] of this file.
    ///
    /// The [`Permissions`] structure can be use as it's more convenient.
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be read.
    fn set_mode(&mut self, mode: Mode) -> Result<(), Error<Self::FsError>>;

    /// Sets the [`Uid`] (identifier of the user owner) of this file.
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be read.
    fn set_uid(&mut self, uid: Uid) -> Result<(), Error<Self::FsError>>;

    /// Sets the [`Gid`] (identifier of the group) of this file.
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be read.
    fn set_gid(&mut self, gid: Gid) -> Result<(), Error<Self::FsError>>;

    /// Sets the `atim` (last data access time) of this file.
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be read.
    fn set_atim(&mut self, atim: Timespec) -> Result<(), Error<Self::FsError>>;

    /// Sets the `mtim` (last data modification time) of this file.
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be read.
    fn set_mtim(&mut self, mtim: Timespec) -> Result<(), Error<Self::FsError>>;

    /// Sets the `ctim` (last file status change time) of this file.
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be read.
    fn set_ctim(&mut self, ctim: Timespec) -> Result<(), Error<Self::FsError>>;
}

/// A readable [`Regular`] file.
///
/// This type can be used alone for read-only filesystems.
pub trait RegularRead: FileRead + Read + Seek {}

/// A file that is a randomly accessible sequence of bytes, with no further structure imposed by the system.
///
/// Defined in [this POSIX definition](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/V1_chap03.html#tag_03_323).
pub trait Regular: File + RegularRead + Write {
    /// Trunctates the file size to the given `size` (in bytes).
    ///
    /// If the given `size` is greater than the previous file size, this function does nothing.
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be read.
    fn truncate(&mut self, size: u64) -> Result<(), Error<<Self as Base>::FsError>>;
}

/// An object that associates a filename with a file. Several directory entries can associate names with the same file.
///
/// Defined in [this POSIX definition](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/V1_chap03.html#tag_03_130).
pub struct DirectoryEntry<'path, Dir: DirectoryRead> {
    /// Name of the file pointed by this directory entry.
    ///
    /// See more information on valid filenames in [this POSIX definition](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/V1_chap03.html#tag_03_170).
    pub filename: UnixStr<'path>,

    /// File pointed by this directory entry.
    pub file: TypeWithFile<Dir>,
}

// A readable [`Directory`] file.
//
/// This type can be used alone for read-only filesystems.
pub trait DirectoryRead: Sized + Base {
    /// Type of the regular files in the [`Filesystem`](crate::fs::Filesystem) this directory belongs to.
    type Regular: RegularRead<FsError = <Self as Base>::FsError>;

    /// Type of the symbolic links in the [`Filesystem`](crate::fs::Filesystem) this directory belongs to.
    type SymbolicLink: SymbolicLinkRead<FsError = <Self as Base>::FsError>;

    /// Type of the fifo in the [`Filesystem`](crate::fs::Filesystem) this directory belongs to.
    type Fifo: FifoRead<FsError = <Self as Base>::FsError>;

    /// Type of the character device in the [`Filesystem`](crate::fs::Filesystem) this directory belongs to.
    type CharacterDevice: CharacterDeviceRead<FsError = <Self as Base>::FsError>;

    /// Type of the character device in the [`Filesystem`](crate::fs::Filesystem) this directory belongs to.
    type BlockDevice: BlockDeviceRead<FsError = <Self as Base>::FsError>;

    /// Type of the UNIX socket in the [`Filesystem`](crate::fs::Filesystem) this directory belongs to.
    type Socket: SocketRead<FsError = <Self as Base>::FsError>;

    /// Returns the directory entries contained.
    ///
    /// No two [`DirectoryEntry`] returned can have the same `filename`.
    ///
    /// The result must contain at least the entries `.` (the current directory) and `..` (the parent directory).
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be read.
    fn entries(&self) -> Result<Vec<DirectoryEntry<'_, Self>>, Error<Self::FsError>>;

    /// Returns the entry with the given name.
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be read.
    fn entry(&self, name: UnixStr) -> Result<Option<TypeWithFile<Self>>, Error<Self::FsError>> {
        let children = self.entries()?;
        Ok(children.into_iter().find(|entry| entry.filename == name).map(|entry| entry.file))
    }

    /// Returns the parent directory.
    ///
    /// If `self` if the root directory, it must return itself.
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be read.
    fn parent(&self) -> Result<Self, Error<Self::FsError>> {
        let Some(TypeWithFile::Directory(parent_entry)) = self.entry(PARENT_DIR.clone())? else {
            unreachable!("`entries` must return `..` that corresponds to the parent directory.")
        };
        Ok(parent_entry)
    }
}

/// A file that contains directory entries. No two directory entries in the same directory have the same name.
///
/// Defined in [this POSIX definition](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/V1_chap03.html#tag_03_129).
pub trait Directory: File + DirectoryRead
where
    <Self as DirectoryRead>::Regular: Regular<FsError = <Self as Base>::FsError>,
    <Self as DirectoryRead>::SymbolicLink: SymbolicLink<FsError = <Self as Base>::FsError>,
    <Self as DirectoryRead>::Fifo: Fifo<FsError = <Self as Base>::FsError>,
    <Self as DirectoryRead>::CharacterDevice: CharacterDevice<FsError = <Self as Base>::FsError>,
    <Self as DirectoryRead>::BlockDevice: BlockDevice<FsError = <Self as Base>::FsError>,
    <Self as DirectoryRead>::Socket: Socket<FsError = <Self as Base>::FsError>,
{
    /// Adds a new empty entry to the directory, meaning that a new file will be created.
    ///
    /// # Errors
    ///
    /// Returns an [`EntryAlreadyExist`](crate::fs::error::FsError::EntryAlreadyExist) error if the entry already exist.
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be written.
    fn add_entry(
        &mut self,
        name: UnixStr<'_>,
        file_type: Type,
        permissions: Permissions,
        user_id: Uid,
        group_id: Gid,
    ) -> Result<TypeWithFile<Self>, Error<Self::FsError>>;

    /// Removes an entry from the directory.
    ///
    /// # Errors
    ///
    /// Returns an [`NotFound`](crate::fs::error::FsError::NotFound) error if there is no entry with the given name in
    /// this directory.
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be written.
    fn remove_entry(&mut self, name: UnixStr) -> Result<(), Error<Self::FsError>>;
}

/// A readable [`SymbolicLink`] file.
///
/// This type can be used alone for read-only filesystems.
pub trait SymbolicLinkRead: FileRead {
    /// Returns the string stored in this symbolic link.
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be read.
    fn get_pointed_file(&self) -> Result<&str, Error<Self::FsError>>;
}

/// A type of file with the property that when the file is encountered during pathname resolution, a string stored by
/// the file is used to modify the pathname resolution.
///
/// Defined in [this POSIX definition](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/V1_chap03.html#tag_03_381).
pub trait SymbolicLink: File + SymbolicLinkRead {
    /// Sets the pointed file in this symbolic link.
    ///
    /// # Errors
    ///
    /// Returns an [`Error::IO`] if the device on which the directory is located could not
    /// be written.
    fn set_pointed_file(&mut self, pointed_file: &str) -> Result<(), Error<Self::FsError>>;
}

/// A readable [`Fifo`] file.
///
/// This type can be used alone for read-only filesystems.
pub trait FifoRead: FileRead {}

/// A type of file with the property that data written to such a file is read on a first-in-first-out basis.
///
/// Defined in [this POSIX defintion](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/V1_chap03.html#tag_03_163)
pub trait Fifo: File + FifoRead {}

/// A readable [`CharacterDevice`] file.
///
/// This type can be used alone for read-only filesystems.
pub trait CharacterDeviceRead: FileRead {}

/// A file that refers to a character device (such as a terminal device file) or that has special properties (such as
/// /dev/null).
///
/// Defined in [this POSIX definition](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/V1_chap03.html#tag_03_91)
pub trait CharacterDevice: File + CharacterDeviceRead {}

/// A readable [`BlockDevice`] file.
///
/// This type can be used alone for read-only filesystems.
pub trait BlockDeviceRead: FileRead {}

/// A file that refers to a block device.
///
/// A block special file is normally distinguished from a character special file by
/// providing access to the device in a manner such that the hardware characteristics of the device are not visible.
///
/// Defined in [this POSIX definition](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/V1_chap03.html#tag_03_79)
pub trait BlockDevice: File + BlockDeviceRead {}

/// A readable [`Socket`] file.
///
/// This type can be used alone for read-only filesystems.
pub trait SocketRead: FileRead {}

/// A file of a particular type that is used as a communications endpoint for process-to-process communication as
/// described in the System Interfaces volume of POSIX.1-2017.
///
/// Defined in [this POSIX definition](https://pubs.opengroup.org/onlinepubs/9799919799/basedefs/V1_chap03.html#tag_03_356)
pub trait Socket: File + SocketRead {}

/// Enumeration of possible file types in a standard UNIX-like filesystem.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Type {
    /// Storage unit of a filesystem.
    Regular,

    /// Node containing other nodes.
    Directory,

    /// Node pointing towards an other node in the filesystem.
    SymbolicLink,

    /// Named pipe.
    Fifo,

    /// An inode that refers to a device communicating by sending chars (bytes) of data.
    CharacterDevice,

    /// An inode that refers to a device communicating by sending blocks of data.
    BlockDevice,

    /// Communication flow between two processes.
    Socket,
}

/// Enumeration of possible file types in a standard UNIX-like filesystem with an attached file object.
#[allow(clippy::module_name_repetitions)]
#[derive(Debug, Clone)]
pub enum TypeWithFile<Dir: DirectoryRead> {
    /// Storage unit of a filesystem.
    Regular(Dir::Regular),

    /// Node containing other nodes.
    Directory(Dir),

    /// Node pointing towards an other node in the filesystem.
    SymbolicLink(Dir::SymbolicLink),

    /// Special node containing a [`Fifo`].
    Fifo(Dir::Fifo),

    /// Special node containing a [`CharacterDevice`].
    CharacterDevice(Dir::CharacterDevice),

    /// Special node containing a [`BlockDevice`].
    BlockDevice(Dir::BlockDevice),

    /// Special node containing a [`Socket`].
    Socket(Dir::Socket),
}

impl<Dir: DirectoryRead> TypeWithFile<Dir> {
    /// Whether this file is a regular file or not.
    pub const fn is_regular(&self) -> bool {
        match self {
            Self::Regular(_) => true,
            Self::Directory(_)
            | Self::SymbolicLink(_)
            | Self::Fifo(_)
            | Self::CharacterDevice(_)
            | Self::BlockDevice(_)
            | Self::Socket(_) => false,
        }
    }

    /// Whether this file is a directory or not.
    pub const fn is_directory(&self) -> bool {
        match self {
            Self::Directory(_) => true,
            Self::Regular(_)
            | Self::SymbolicLink(_)
            | Self::Fifo(_)
            | Self::CharacterDevice(_)
            | Self::BlockDevice(_)
            | Self::Socket(_) => false,
        }
    }

    /// Whether this file is a symbolic link or not.
    pub const fn is_symlink(&self) -> bool {
        match self {
            Self::SymbolicLink(_) => true,
            Self::Regular(_)
            | Self::Directory(_)
            | Self::Fifo(_)
            | Self::CharacterDevice(_)
            | Self::BlockDevice(_)
            | Self::Socket(_) => false,
        }
    }

    /// Whether this file is a fifo or not.
    pub const fn is_fifo(&self) -> bool {
        match self {
            Self::Fifo(_) => true,
            Self::Regular(_)
            | Self::Directory(_)
            | Self::SymbolicLink(_)
            | Self::CharacterDevice(_)
            | Self::BlockDevice(_)
            | Self::Socket(_) => false,
        }
    }

    /// Whether this file is a character device or not.
    pub const fn is_character_device(&self) -> bool {
        match self {
            Self::CharacterDevice(_) => true,
            Self::Regular(_)
            | Self::Directory(_)
            | Self::SymbolicLink(_)
            | Self::Fifo(_)
            | Self::BlockDevice(_)
            | Self::Socket(_) => false,
        }
    }

    /// Whether this file is a block device or not.
    pub const fn is_block_device(&self) -> bool {
        match self {
            Self::BlockDevice(_) => true,
            Self::Regular(_)
            | Self::Directory(_)
            | Self::SymbolicLink(_)
            | Self::Fifo(_)
            | Self::CharacterDevice(_)
            | Self::Socket(_) => false,
        }
    }

    /// Whether this file is a UNIX socket or not.
    pub const fn is_socket(&self) -> bool {
        match self {
            Self::Socket(_) => true,
            Self::Regular(_)
            | Self::Directory(_)
            | Self::SymbolicLink(_)
            | Self::Fifo(_)
            | Self::CharacterDevice(_)
            | Self::BlockDevice(_) => false,
        }
    }
}

impl<Dir: DirectoryRead> From<&TypeWithFile<Dir>> for Type {
    fn from(value: &TypeWithFile<Dir>) -> Self {
        match value {
            TypeWithFile::Regular(_) => Self::Regular,
            TypeWithFile::Directory(_) => Self::Directory,
            TypeWithFile::SymbolicLink(_) => Self::SymbolicLink,
            TypeWithFile::Fifo(_) => Self::Fifo,
            TypeWithFile::CharacterDevice(_) => Self::CharacterDevice,
            TypeWithFile::BlockDevice(_) => Self::BlockDevice,
            TypeWithFile::Socket(_) => Self::Socket,
        }
    }
}

impl<Dir: DirectoryRead> From<TypeWithFile<Dir>> for Type {
    fn from(value: TypeWithFile<Dir>) -> Self {
        match value {
            TypeWithFile::Regular(_) => Self::Regular,
            TypeWithFile::Directory(_) => Self::Directory,
            TypeWithFile::SymbolicLink(_) => Self::SymbolicLink,
            TypeWithFile::Fifo(_) => Self::Fifo,
            TypeWithFile::CharacterDevice(_) => Self::CharacterDevice,
            TypeWithFile::BlockDevice(_) => Self::BlockDevice,
            TypeWithFile::Socket(_) => Self::Socket,
        }
    }
}
