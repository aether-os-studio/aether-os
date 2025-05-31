use alloc::{collections::btree_map::BTreeMap, ffi::CString, vec::Vec};
use rmm::{Arch, PageFlags, PageMapper, VirtualAddress};

use core::ptr::null;

use crate::arch::CurrentMMArch;
use crate::{errno::Errno, memory::frame::TheFrameAllocator, syscall::Result};

use super::sched::get_current_context;

#[derive(Debug)]
pub struct ProcInitInfo {
    pub proc_name: CString,
    pub args: Vec<CString>,
    pub envs: Vec<CString>,
    pub auxv: BTreeMap<u8, usize>,
}

impl ProcInitInfo {
    pub fn new(proc_name: &str) -> Self {
        Self {
            proc_name: CString::new(proc_name).unwrap_or(CString::new("").unwrap()),
            args: Vec::new(),
            envs: Vec::new(),
            auxv: BTreeMap::new(),
        }
    }

    pub unsafe fn push_at(&self, mut ustack: usize) -> Result<(usize, usize)> {
        self.push_str(ustack, &self.proc_name)?;

        let envps = self
            .envs
            .iter()
            .map(|s| {
                ustack = self.push_str(ustack, s).expect("push_str failed");
                ustack
            })
            .collect::<Vec<_>>();
        let argps = self
            .args
            .iter()
            .map(|s| {
                ustack = self.push_str(ustack, s).expect("push_str failed");
                ustack
            })
            .collect::<Vec<_>>();

        // 压入auxv
        ustack = self.push_slice(ustack, &[null::<u8>(), null::<u8>()])?;
        for (&k, &v) in self.auxv.iter() {
            ustack = self.push_slice(ustack, &[k as usize, v])?;
        }

        // 把环境变量指针压入栈中
        ustack = self.push_slice(ustack, &[null::<u8>()])?;
        ustack = self.push_slice(ustack, envps.as_slice())?;

        // 把参数指针压入栈中
        ustack = self.push_slice(ustack, &[null::<u8>()])?;
        ustack = self.push_slice(ustack, argps.as_slice())?;

        let argv_ptr = ustack;

        // 把argc压入栈中
        ustack = self.push_slice(ustack, &[self.args.len()])?;

        return Ok((ustack, argv_ptr));
    }

    fn push_slice<T: Copy>(&self, ustack: usize, slice: &[T]) -> Result<usize> {
        let mut sp = ustack;
        sp -= core::mem::size_of_val(slice);
        sp -= sp % core::mem::align_of::<T>();

        unsafe { core::slice::from_raw_parts_mut(sp as *mut T, slice.len()) }
            .copy_from_slice(slice);

        return Ok(sp);
    }

    fn push_str(&self, ustack: usize, s: &CString) -> Result<usize> {
        let bytes = s.as_bytes_with_nul();
        let ustack = self.push_slice(ustack, bytes)?;
        return Ok(ustack);
    }
}

use goblin::elf::{Elf, program_header::PT_LOAD};

pub fn execve_from_buffer(
    buffer: &'static [u8],
    argv: *const *mut core::ffi::c_char,
    envp: *const *mut core::ffi::c_char,
) -> Result<()> {
    let mut args = Vec::new();
    unsafe {
        if !argv.is_null() {
            let mut i = 0;
            while !(*argv.add(i)).is_null() {
                let cstr = CString::from_raw(*argv.add(i));
                args.push(cstr);
                i += 1;
            }
        }
    }

    let mut envs = Vec::new();
    unsafe {
        if !envp.is_null() {
            let mut i = 0;
            while !(*envp.add(i)).is_null() {
                let cstr = CString::from_raw(*envp.add(i));
                envs.push(cstr);
                i += 1;
            }
        }
    }

    let elf_header = Elf::parse(buffer).or(Err(Errno::EINVAL))?;

    let mut mapper = unsafe { PageMapper::current(rmm::TableKind::User, TheFrameAllocator) };

    for phdr in elf_header.program_headers.iter() {
        if phdr.p_type != PT_LOAD {
            continue;
        }

        let vaddr = phdr.p_vaddr;
        let offset = phdr.p_offset;
        let size = phdr.p_filesz;
        let memsize = phdr.p_memsz;

        let page_table_flags = PageFlags::<CurrentMMArch>::new()
            .execute(phdr.is_executable())
            .write(phdr.is_write());

        let aligned_vaddr =
            (vaddr as usize + CurrentMMArch::PAGE_SIZE - 1) & !(CurrentMMArch::PAGE_SIZE - 1);
        let count = (size as usize + CurrentMMArch::PAGE_SIZE - 1) / CurrentMMArch::PAGE_SIZE;

        for i in 0..count {
            let result = unsafe {
                mapper.map(
                    VirtualAddress::new(aligned_vaddr as usize + i * CurrentMMArch::PAGE_SIZE),
                    page_table_flags,
                )
            };
            if let Some(flusher) = result {
                flusher.flush();
            } else {
                return Err(Errno::ENOMEM);
            }
        }

        // todo: load interpreter

        unsafe { core::slice::from_raw_parts_mut(vaddr as *mut u8, size as usize) }
            .copy_from_slice(unsafe {
                core::slice::from_raw_parts(
                    (buffer.as_ptr() as u64 + offset) as *const u8,
                    size as usize,
                )
            });
    }

    let mut proc_init_info = ProcInitInfo::new(&get_current_context().read().name);
    proc_init_info.args = args;
    proc_init_info.envs = envs;

    get_current_context().write().init_info = Some(proc_init_info);

    Ok(())
}
