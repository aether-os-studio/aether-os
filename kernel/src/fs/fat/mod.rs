use core::usize;

use alloc::{boxed::Box, collections::BTreeMap, string::String, sync::Arc};
use fatfs::*;
use spin::RwLock;

use crate::{errno::Errno, syscall::Result};

use super::{
    path_walk::{get_inode_by_path, ref_to_mut, ref_to_static},
    vfs::{Dirent, IndexNode, IndexNodeInfo, IndexNodeRef, IndexNodeType},
};

type FatDir = Dir<'static, IndexNodeRefIO, NullTimeProvider, LossyOemCpConverter>;
type FatFile = File<'static, IndexNodeRefIO, NullTimeProvider, LossyOemCpConverter>;

pub struct IndexNodeRefIO {
    inode: IndexNodeRef,
    offset: usize,
}

impl IndexNodeRefIO {
    pub fn new(inode: IndexNodeRef) -> Self {
        Self { inode, offset: 0 }
    }
}

impl IoBase for IndexNodeRefIO {
    type Error = ();
}

impl Read for IndexNodeRefIO {
    fn read(&mut self, buf: &mut [u8]) -> core::result::Result<usize, Self::Error> {
        self.inode.read().read_at(self.offset, buf).or(Err(()))?;
        self.seek(SeekFrom::Current(buf.len() as i64))?;
        Ok(buf.len())
    }
}

impl Write for IndexNodeRefIO {
    fn write(&mut self, buf: &[u8]) -> core::result::Result<usize, Self::Error> {
        self.inode.read().write_at(self.offset, buf).or(Err(()))?;
        self.seek(SeekFrom::Current(buf.len() as i64))?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> core::result::Result<(), Self::Error> {
        Ok(())
    }
}

impl Seek for IndexNodeRefIO {
    fn seek(&mut self, pos: SeekFrom) -> core::result::Result<u64, Self::Error> {
        match pos {
            SeekFrom::Current(i) => self.offset = (self.offset as i64 + i) as usize,
            SeekFrom::Start(i) => self.offset = i as usize,
            SeekFrom::End(i) => {
                let size = self.inode.read().get_info().size;
                self.offset = size - i as usize;
            }
        }
        Ok(self.offset as u64)
    }
}

pub struct Fat32Volume {
    vol: &'static mut FileSystem<IndexNodeRefIO, NullTimeProvider, LossyOemCpConverter>,
    virtual_inodes: BTreeMap<String, IndexNodeRef>,
    path: String,
}

impl Fat32Volume {
    pub fn new(dev: IndexNodeRef) -> Option<IndexNodeRef> {
        let io = IndexNodeRefIO::new(dev);
        let vol = FileSystem::new(io, FsOptions::new());
        if vol.is_err() {
            return None;
        }

        let inode = Self {
            vol: Box::leak(Box::new(vol.unwrap())),
            virtual_inodes: BTreeMap::new(),
            path: String::new(),
        };

        let inode_ref = Arc::new(RwLock::new(inode));
        ref_to_mut(&*inode_ref.read())
            .virtual_inodes
            .insert(".".into(), inode_ref.clone());
        Some(inode_ref)
    }
}

impl IndexNode for Fat32Volume {
    fn when_mounted(&mut self, path: alloc::string::String, father: Option<IndexNodeRef>) {
        self.path.clear();
        self.path.push_str(path.as_str());
        if let Some(father) = father {
            self.virtual_inodes.insert("..".into(), father);
        }
    }

    fn when_umounted(&mut self) {}

    fn get_info(&self) -> IndexNodeInfo {
        let mut info = IndexNodeInfo::default();
        info.path = self.path.clone();
        info.inode_type = IndexNodeType::Dir;
        info
    }

    fn mount(&self, node: IndexNodeRef, name: String) -> Result<()> {
        ref_to_mut(self)
            .virtual_inodes
            .insert(name.clone(), node.clone());
        Ok(())
    }

    fn open(&self, name: String) -> Result<IndexNodeRef> {
        let cluster_size = self.vol.cluster_size() as usize;
        let dir = ref_to_static(self).vol.root_dir();

        let self_inode = get_inode_by_path(self.get_info().path);

        if let Some(inode) = self.virtual_inodes.get(&name) {
            return Ok(inode.clone());
        } else if let Ok(dir) = dir.open_dir(name.as_str()) {
            let inode = Fat32Dir::new(Arc::new(dir), cluster_size);
            inode
                .write()
                .when_mounted(self.get_info().path + name.as_str() + "/", self_inode);
            return Ok(inode);
        } else if let Ok(file) = dir.open_file(name.as_str()) {
            let inode = Arc::new(RwLock::new(Fat32File::new(Arc::new(file), cluster_size)));
            inode
                .write()
                .when_mounted(self.get_info().path + name.as_str(), self_inode);
            return Ok(inode);
        }

        Err(Errno::ENOENT)
    }

    fn create(&self, name: String, ty: IndexNodeType) -> Result<IndexNodeRef> {
        match ty {
            IndexNodeType::Dir => {
                self.vol
                    .root_dir()
                    .create_dir(name.as_str())
                    .or(Err(Errno::EEXIST))?;
            }
            IndexNodeType::File => {
                self.vol
                    .root_dir()
                    .create_file(name.as_str())
                    .or(Err(Errno::EEXIST))?;
            }
            _ => return Err(Errno::ENOSYS),
        }
        self.open(name)
    }

    fn ioctl(&self, _cmd: usize, _arg: usize) -> Result<usize> {
        Ok(0)
    }

    fn getdents(&self, dents: &mut [Dirent]) -> Result<()> {
        Ok(())
    }
}

pub struct Fat32Dir {
    dir: Arc<FatDir>,
    path: String,
    cluster_size: usize,
    virtual_inodes: BTreeMap<String, IndexNodeRef>,
}

impl Fat32Dir {
    pub(self) fn new(dir: Arc<FatDir>, cluster_size: usize) -> IndexNodeRef {
        let inode = Self {
            dir,
            path: String::new(),
            cluster_size,
            virtual_inodes: BTreeMap::new(),
        };
        let inode_ref = Arc::new(RwLock::new(inode));
        ref_to_mut(&*inode_ref.read())
            .virtual_inodes
            .insert(".".into(), inode_ref.clone());
        inode_ref
    }
}

impl IndexNode for Fat32Dir {
    fn when_mounted(&mut self, path: alloc::string::String, father: Option<IndexNodeRef>) {
        self.path.clear();
        self.path.push_str(path.as_str());
        if let Some(father) = father {
            self.virtual_inodes.insert("..".into(), father);
        }
    }

    fn when_umounted(&mut self) {}

    fn get_info(&self) -> IndexNodeInfo {
        let mut info = IndexNodeInfo::default();
        info.path = self.path.clone();
        info.inode_type = IndexNodeType::Dir;
        info
    }

    fn mount(&self, node: IndexNodeRef, name: String) -> Result<()> {
        ref_to_mut(self)
            .virtual_inodes
            .insert(name.clone(), node.clone());
        Ok(())
    }

    fn open(&self, name: String) -> Result<IndexNodeRef> {
        let self_inode = get_inode_by_path(self.get_info().path);

        if let Some(inode) = self.virtual_inodes.get(&name) {
            return Ok(inode.clone());
        } else if let Ok(dir) = self.dir.open_dir(name.as_str()) {
            let inode = Fat32Dir::new(Arc::new(dir), self.cluster_size);
            inode
                .write()
                .when_mounted(self.get_info().path + name.as_str() + "/", self_inode);
            return Ok(inode);
        } else if let Ok(file) = self.dir.open_file(name.as_str()) {
            let inode = Arc::new(RwLock::new(Fat32File::new(
                Arc::new(file),
                self.cluster_size,
            )));
            inode
                .write()
                .when_mounted(self.get_info().path + name.as_str(), self_inode);
            return Ok(inode);
        }

        Err(Errno::ENOENT)
    }

    fn create(&self, name: String, ty: IndexNodeType) -> Result<IndexNodeRef> {
        match ty {
            IndexNodeType::Dir => {
                self.dir.create_dir(name.as_str()).or(Err(Errno::EEXIST))?;
            }
            IndexNodeType::File => {
                self.dir.create_file(name.as_str()).or(Err(Errno::EEXIST))?;
            }
            _ => return Err(Errno::ENOSYS),
        }
        self.open(name)
    }

    fn ioctl(&self, _cmd: usize, _arg: usize) -> Result<usize> {
        Ok(0)
    }

    fn getdents(&self, dents: &mut [Dirent]) -> Result<()> {
        Ok(())
    }
}

pub struct Fat32File {
    file: Arc<FatFile>,
    path: String,
    cluster_size: usize,
}

impl Fat32File {
    pub(self) fn new(file: Arc<FatFile>, cluster_size: usize) -> Self {
        Self {
            file,
            path: String::new(),
            cluster_size,
        }
    }
}

impl IndexNode for Fat32File {
    fn when_mounted(&mut self, path: alloc::string::String, _father: Option<IndexNodeRef>) {
        self.path.clear();
        self.path.push_str(path.as_str());
    }

    fn when_umounted(&mut self) {}

    fn get_info(&self) -> IndexNodeInfo {
        let mut info = IndexNodeInfo::default();
        info.path = self.path.clone();
        info.inode_type = IndexNodeType::File;
        info.size = self.file.size().unwrap() as usize;
        info
    }

    fn read_at(&self, offset: usize, buf: &mut [u8]) -> Result<usize> {
        let mut size = 0;

        ref_to_mut(self.file.as_ref())
            .seek(SeekFrom::Start(offset as u64))
            .unwrap();
        let read_size = buf.len().min(self.get_info().size);

        let read_cnt = read_size / self.cluster_size;

        for i in 0..read_cnt {
            let offset = self.cluster_size * i;
            size += ref_to_mut(self.file.as_ref())
                .read(&mut buf[offset..offset + self.cluster_size])
                .unwrap();
        }

        let remaining = read_size % self.cluster_size;

        if remaining > 0 && buf.len() <= self.get_info().size {
            size += ref_to_mut(self.file.as_ref())
                .read(&mut buf[read_cnt * self.cluster_size..])
                .unwrap();
        }

        Ok(size)
    }

    fn write_at(&self, offset: usize, buf: &[u8]) -> Result<usize> {
        let mut size = 0;
        ref_to_mut(self.file.as_ref())
            .seek(SeekFrom::Start(offset as u64))
            .unwrap();
        let write_size = buf.len();

        let write_cnt = write_size / self.cluster_size;

        for i in 0..write_cnt {
            let offset = self.cluster_size * i;
            size += ref_to_mut(self.file.as_ref())
                .write(&buf[offset..offset + self.cluster_size])
                .unwrap();
        }

        let remaining = write_size % self.cluster_size;
        if remaining > 0 {
            size += ref_to_mut(self.file.as_ref())
                .write(&buf[write_cnt * self.cluster_size..])
                .unwrap();
        }

        ref_to_mut(self.file.as_ref()).flush().unwrap();

        Ok(size)
    }

    fn ioctl(&self, _cmd: usize, _arg: usize) -> Result<usize> {
        Ok(0)
    }
}

unsafe impl Send for Fat32Volume {}
unsafe impl Send for Fat32Dir {}
unsafe impl Send for Fat32File {}

unsafe impl Sync for Fat32Volume {}
unsafe impl Sync for Fat32Dir {}
unsafe impl Sync for Fat32File {}
