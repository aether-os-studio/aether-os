use core::ffi::CStr;

use alloc::string::ToString;

use crate::{errno::Errno, syscall::Result};

use super::{fd::get_file_descriptor_manager, path_walk::get_inode_by_path};

pub fn sys_open(
    path: *const core::ffi::c_char,
    flags: core::ffi::c_int,
    mode: core::ffi::c_int,
) -> Result<usize> {
    let path = unsafe { CStr::from_ptr(path).to_str().unwrap() }.to_string();

    let current_file_descriptor_manager = get_file_descriptor_manager().ok_or(Errno::ENOENT)?;

    let inode = if path.starts_with("/") {
        get_inode_by_path(path.clone()).ok_or(Errno::ENOENT)?
    } else {
        get_inode_by_path(alloc::format!(
            "{}{}",
            current_file_descriptor_manager.get_cwd(),
            path.clone()
        ))
        .ok_or(Errno::ENOENT)?
    };

    let file_descriptor =
        current_file_descriptor_manager.add_inode(inode, super::fd::OpenMode::ReadWrite); // TODO: mode

    Ok(file_descriptor)
}
