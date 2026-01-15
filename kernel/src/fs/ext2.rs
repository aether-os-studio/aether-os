use core::str::FromStr;

use alloc::{
    string::{String, ToString},
    sync::Arc,
    vec::Vec,
};
use deku::no_std_io::{ErrorKind, Read, Seek, Write};
use efs::{
    fs::{
        FilesystemRead,
        ext2::Ext2Fs,
        file::{DirectoryRead, File, FileRead},
        types::{Gid, Mode, Uid},
    },
    path::Path,
};
use spin::RwLock;

use crate::{
    drivers::storage::BlockDevice,
    fs::vfs::{
        dirent::Dirent,
        file::{ArcFile, FileTrait, FileType},
        fs::{ArcFileSystem, FileSystemTrait},
    },
};

impl efs::dev::Device for BlockDevice {
    fn size(&mut self) -> efs::dev::size::Size {
        efs::dev::size::Size(self.block_count() * (self.block_size() as u64))
    }

    fn slice(
        &mut self,
        addr_range: core::ops::Range<efs::dev::address::Address>,
    ) -> deku::no_std_io::Result<efs::dev::Slice<'_>> {
        let mut buf = vec![0; addr_range.end.index() as usize - addr_range.start.index() as usize];
        self.read_block(addr_range.start.index(), &mut buf)
            .map_err(|e| deku::no_std_io::Error::new(ErrorKind::InvalidInput, e))?;
        Ok(efs::dev::Slice::new_owned(buf, addr_range.start))
    }

    fn commit(&mut self, commit: efs::dev::Commit) -> deku::no_std_io::Result<()> {
        self.write_block(commit.addr().index(), commit.as_ref())
            .map_err(|e| deku::no_std_io::Error::new(ErrorKind::InvalidInput, e))
    }
}

pub struct Ext2File {
    file: efs::fs::ext2::Ext2TypeWithFile<BlockDevice>,
}

impl Ext2File {
    pub fn new(file: efs::fs::ext2::Ext2TypeWithFile<BlockDevice>) -> Self {
        Self { file }
    }
}

impl Ext2File {
    fn file_info(&self) -> Option<(FileType, u64)> {
        if let efs::fs::ext2::Ext2TypeWithFile::Regular(regular) = self.file.clone() {
            let stat = regular.stat();
            Some((FileType::Regular, stat.ino.0))
        } else if let efs::fs::ext2::Ext2TypeWithFile::Directory(directory) = self.file.clone() {
            let stat = directory.stat();
            Some((FileType::Directory, stat.ino.0))
        } else if let efs::fs::ext2::Ext2TypeWithFile::SymbolicLink(symlink) = self.file.clone() {
            let stat = symlink.stat();
            Some((FileType::Symlink, stat.ino.0))
        } else if let efs::fs::ext2::Ext2TypeWithFile::Fifo(fifo) = self.file.clone() {
            let stat = fifo.stat();
            Some((FileType::Fifo, stat.ino.0))
        } else if let efs::fs::ext2::Ext2TypeWithFile::CharacterDevice(chardev) = self.file.clone()
        {
            let stat = chardev.stat();
            Some((FileType::CharDev, stat.ino.0))
        } else if let efs::fs::ext2::Ext2TypeWithFile::BlockDevice(blkdev) = self.file.clone() {
            let stat = blkdev.stat();
            Some((FileType::BlockDev, stat.ino.0))
        } else if let efs::fs::ext2::Ext2TypeWithFile::Socket(socket) = self.file.clone() {
            let stat = socket.stat();
            Some((FileType::Socket, stat.ino.0))
        } else {
            None
        }
    }
}

impl FileTrait for Ext2File {
    fn size(&self) -> Option<usize> {
        if let efs::fs::ext2::Ext2TypeWithFile::Regular(regular) = self.file.clone() {
            Some(regular.stat().size.0 as usize)
        } else {
            None
        }
    }

    fn read(&mut self, buf: &mut [u8], offset: usize) -> Option<usize> {
        if let efs::fs::ext2::Ext2TypeWithFile::Regular(mut regular) = self.file.clone() {
            regular
                .seek(deku::no_std_io::SeekFrom::Start(offset as u64))
                .ok()?;
            regular.read(buf).ok()
        } else {
            None
        }
    }

    fn write(&mut self, buf: &[u8], offset: usize) -> Option<usize> {
        if let efs::fs::ext2::Ext2TypeWithFile::Regular(mut regular) = self.file.clone() {
            regular
                .seek(deku::no_std_io::SeekFrom::Start(offset as u64))
                .ok()?;
            regular.write(buf).ok()
        } else {
            None
        }
    }

    fn readdir(&mut self) -> Option<Vec<Dirent>> {
        if let efs::fs::ext2::Ext2TypeWithFile::Directory(directory) = self.file.clone() {
            let entries = directory.entries().ok()?;
            let mut result = Vec::new();
            for entry in entries {
                let child = Ext2File::new(entry.file);
                let (filetype, inode) = child.file_info()?;
                let dirent = Dirent::new(inode, filetype, entry.filename.to_string());
                result.push(dirent);
            }
            Some(result)
        } else {
            None
        }
    }

    fn chmod(&mut self, mode: u16) -> Option<()> {
        let mode = Mode(mode);
        if let efs::fs::ext2::Ext2TypeWithFile::Regular(mut regular) = self.file.clone() {
            regular.set_mode(mode).ok()?;
            Some(())
        } else if let efs::fs::ext2::Ext2TypeWithFile::Directory(mut directory) = self.file.clone()
        {
            directory.set_mode(mode).ok()?;
            Some(())
        } else if let efs::fs::ext2::Ext2TypeWithFile::SymbolicLink(mut symlink) = self.file.clone()
        {
            symlink.set_mode(mode).ok()?;
            Some(())
        } else if let efs::fs::ext2::Ext2TypeWithFile::Fifo(mut fifo) = self.file.clone() {
            fifo.set_mode(mode).ok()?;
            Some(())
        } else if let efs::fs::ext2::Ext2TypeWithFile::CharacterDevice(mut chardev) =
            self.file.clone()
        {
            chardev.set_mode(mode).ok()?;
            Some(())
        } else if let efs::fs::ext2::Ext2TypeWithFile::BlockDevice(mut blkdev) = self.file.clone() {
            blkdev.set_mode(mode).ok()?;
            Some(())
        } else if let efs::fs::ext2::Ext2TypeWithFile::Socket(mut socket) = self.file.clone() {
            socket.set_mode(mode).ok()?;
            Some(())
        } else {
            None
        }
    }

    fn chown(&mut self, uid: u32, gid: u32) -> Option<()> {
        let uid = Uid(uid);
        let gid = Gid(gid);

        if let efs::fs::ext2::Ext2TypeWithFile::Regular(mut regular) = self.file.clone() {
            regular.set_uid(uid).ok()?;
            regular.set_gid(gid).ok()?;
            Some(())
        } else if let efs::fs::ext2::Ext2TypeWithFile::Directory(mut directory) = self.file.clone()
        {
            directory.set_uid(uid).ok()?;
            directory.set_gid(gid).ok()?;
            Some(())
        } else if let efs::fs::ext2::Ext2TypeWithFile::SymbolicLink(mut symlink) = self.file.clone()
        {
            symlink.set_uid(uid).ok()?;
            symlink.set_gid(gid).ok()?;
            Some(())
        } else if let efs::fs::ext2::Ext2TypeWithFile::Fifo(mut fifo) = self.file.clone() {
            fifo.set_uid(uid).ok()?;
            fifo.set_gid(gid).ok()?;
            Some(())
        } else if let efs::fs::ext2::Ext2TypeWithFile::CharacterDevice(mut chardev) =
            self.file.clone()
        {
            chardev.set_uid(uid).ok()?;
            chardev.set_gid(gid).ok()?;
            Some(())
        } else if let efs::fs::ext2::Ext2TypeWithFile::BlockDevice(mut blkdev) = self.file.clone() {
            blkdev.set_uid(uid).ok()?;
            blkdev.set_gid(gid).ok()?;
            Some(())
        } else if let efs::fs::ext2::Ext2TypeWithFile::Socket(mut socket) = self.file.clone() {
            socket.set_uid(uid).ok()?;
            socket.set_gid(gid).ok()?;
            Some(())
        } else {
            None
        }
    }
}

pub struct Ext2FileSystem {
    fs: Ext2Fs<BlockDevice>,
}

impl Ext2FileSystem {
    pub fn new(dev: BlockDevice) -> Option<ArcFileSystem> {
        let fs = Ext2Fs::new(dev, 0).ok()?;

        Some(Arc::new(RwLock::new(Self { fs })))
    }
}

impl FileSystemTrait for Ext2FileSystem {
    fn lookup(&mut self, path: String, follow_symlink: bool) -> Option<ArcFile> {
        let e2file = self
            .fs
            .get_file(
                &Path::from_str(&path).unwrap(),
                self.fs.root().expect("Failed get root"),
                follow_symlink,
            )
            .ok()?;
        Some(Arc::new(RwLock::new(Ext2File::new(e2file))))
    }
}
