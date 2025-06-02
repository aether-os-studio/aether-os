use core::sync::atomic::{AtomicUsize, Ordering};

use alloc::{collections::btree_map::BTreeMap, string::String, sync::Arc};
use spin::Mutex;

use crate::proc::{MAX_FD_NUM, sched::get_current_context};

use super::{
    ROOT,
    path_walk::{get_inode_by_path, ref_to_mut},
    vfs::{IndexNodeRef, IndexNodeType},
};

pub static FILE_DESCRIPTOR_MANAGERS: Mutex<BTreeMap<usize, Arc<FileDescriptorManager>>> =
    Mutex::new(BTreeMap::new());

#[derive(Debug, Clone, Copy)]
pub enum OpenMode {
    Read = 0,
    Write,
    ReadWrite,
}

type FileDescriptor = usize;
type FileTuple = (IndexNodeRef, OpenMode, usize);

pub struct FileDescriptorManager {
    pub file_descriptors: BTreeMap<FileDescriptor, FileTuple>,
    file_descriptor_allocator: AtomicUsize,
    cwd: Mutex<IndexNodeRef>,
}

impl FileDescriptorManager {
    pub fn new(file_descriptors: BTreeMap<FileDescriptor, FileTuple>) -> Self {
        Self {
            file_descriptors,
            file_descriptor_allocator: AtomicUsize::new(3), // 0, 1, and 2 are reserved for stdin, stdout, and stderr
            cwd: Mutex::new(ROOT.lock().clone()),
        }
    }

    pub fn get_new_fd(&self) -> FileDescriptor {
        let ret = self
            .file_descriptor_allocator
            .fetch_add(1, Ordering::SeqCst);
        if self.file_descriptor_allocator.load(Ordering::SeqCst) >= MAX_FD_NUM {
            self.file_descriptor_allocator.store(3, Ordering::SeqCst);
        }
        ret
    }

    pub fn add_inode(&self, inode: IndexNodeRef, mode: OpenMode) -> FileDescriptor {
        let new_fd = self.get_new_fd();
        ref_to_mut(self)
            .file_descriptors
            .insert(new_fd, (inode, mode, 0));
        new_fd
    }

    pub fn change_cwd(&self, path: String) {
        if let Some(inode) = get_inode_by_path(path) {
            if inode.read().get_info().inode_type == IndexNodeType::Dir {
                *self.cwd.lock() = inode;
            }
        }
    }

    pub fn get_cwd(&self) -> String {
        self.cwd.lock().read().get_info().path.clone()
    }
}

pub fn get_file_descriptor_manager() -> Option<Arc<FileDescriptorManager>> {
    let pid = get_current_context().read().get_pid();
    FILE_DESCRIPTOR_MANAGERS.lock().get_mut(&pid).cloned()
}

pub fn init_file_descriptor_manager(pid: usize) {
    let mut file_descriptor_managers = FILE_DESCRIPTOR_MANAGERS.lock();
    file_descriptor_managers.insert(pid, Arc::new(FileDescriptorManager::new(BTreeMap::new())));
}

pub fn init_file_descriptor_manager_with_stdin_stdout(
    pid: usize,
    stdin: IndexNodeRef,
    stdout: IndexNodeRef,
) {
    let mut file_descriptor_managers = FILE_DESCRIPTOR_MANAGERS.lock();

    let mut file_descriptors = BTreeMap::new();
    file_descriptors.insert(0, (stdin.clone(), OpenMode::Read, 0));
    file_descriptors.insert(1, (stdout.clone(), OpenMode::Write, 0));
    file_descriptors.insert(2, (stdout.clone(), OpenMode::Write, 0));

    file_descriptor_managers.insert(pid, Arc::new(FileDescriptorManager::new(file_descriptors)));
}
