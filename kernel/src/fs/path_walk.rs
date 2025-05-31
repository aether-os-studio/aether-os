use alloc::string::String;

use super::{ROOT, vfs::IndexNodeRef};

pub fn addr_of<T>(reffer: &T) -> usize {
    reffer as *const T as usize
}

pub fn ref_to_mut<T>(reffer: &T) -> &mut T {
    unsafe { &mut *(addr_of(reffer) as *const T as *mut T) }
}

pub fn ref_to_static<T>(reffer: &T) -> &'static T {
    unsafe { &*(addr_of(reffer) as *const T) }
}

pub fn get_inode_by_path(path: String) -> Option<IndexNodeRef> {
    let root = ROOT.lock().clone();

    let path = path.split("/");

    let node = root;

    for path_node in path {
        if path_node.len() > 0 {
            if let Ok(child) = node.read().open(String::from(path_node)) {
                core::mem::drop(core::mem::replace(ref_to_mut(&node), child));
            } else {
                return None;
            }
        }
    }

    Some(node.clone())
}
