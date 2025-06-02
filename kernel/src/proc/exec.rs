use alloc::string::ToString;
use alloc::{collections::btree_map::BTreeMap, ffi::CString, vec::Vec};
use goblin::elf::ProgramHeader;
use rmm::{Arch, PageFlags, PageMapper, VirtualAddress};

use core::ffi::CStr;
use core::ptr::null;
use core::usize;

use crate::arch::proc::context::arch_to_user_mode;
use crate::arch::{CurrentMMArch, arch_disable_intr};
use crate::fs::path_walk::get_inode_by_path;
use crate::{errno::Errno, memory::frame::TheFrameAllocator, syscall::Result};

use super::abi::AtType;
use super::sched::get_current_context;

pub const USER_STACK_END: usize = 0x0000_7000_0000_0000;
pub const USER_STACK_SIZE: usize = 0x0000_0000_0100_0000;

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
        ustack = self.push_str(ustack, &self.proc_name)?;
        let execfn_ptr = ustack;

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

        ustack = self.push_slice(ustack, &[null::<u8>(), null::<u8>()])?;
        for (&k, &v) in self.auxv.iter() {
            ustack = self.push_slice(ustack, &[k as usize, v])?;
        }
        ustack = self.push_slice(ustack, &[AtType::EXECFN as usize, execfn_ptr as usize])?;

        ustack = self.push_slice(ustack, &[null::<u8>()])?;
        ustack = self.push_slice(ustack, envps.as_slice())?;

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

    fn push_str(&self, mut ustack: usize, s: &CString) -> Result<usize> {
        let bytes = s.as_bytes_with_nul();
        ustack = self.push_slice(ustack, bytes)?;
        return Ok(ustack);
    }
}

use goblin::elf::{Elf, program_header::PT_LOAD};

pub fn execve_from_buffer(
    path: *const core::ffi::c_char,
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

    let mut use_ldso: bool = false;
    let mut ldso_entry: usize = 0;

    let elf_header = Elf::parse(buffer).or(Err(Errno::EINVAL))?;

    let mut mapper = unsafe { PageMapper::current(rmm::TableKind::User, TheFrameAllocator) };

    let mut load_start = usize::MAX;
    let mut load_end = 0;
    let mut interpreter_load_start = usize::MAX;
    let mut interpreter_load_end = 0;

    for phdr in elf_header.program_headers.iter() {
        let vaddr = phdr.p_vaddr;
        let offset = phdr.p_offset;
        let size = phdr.p_filesz;
        let memsize = phdr.p_memsz;

        if (vaddr as usize) < load_start {
            load_start = vaddr as usize;
        }
        if (vaddr as usize + memsize as usize) > load_end {
            load_end = vaddr as usize + memsize as usize;
        }

        if phdr.p_type != PT_LOAD {
            continue;
        }

        let page_table_flags = PageFlags::<CurrentMMArch>::new()
            .execute(true)
            .write(true)
            .user(true);

        let aligned_vaddr = (vaddr as usize) & !CurrentMMArch::PAGE_OFFSET_MASK;
        let aligned_end = (vaddr as usize + memsize as usize + CurrentMMArch::PAGE_SIZE - 1)
            & !(CurrentMMArch::PAGE_OFFSET_MASK);

        let count =
            (aligned_end - aligned_vaddr + CurrentMMArch::PAGE_SIZE - 1) / CurrentMMArch::PAGE_SIZE;

        for i in 0..count {
            let result = unsafe {
                mapper.map(
                    VirtualAddress::new(aligned_vaddr as usize + i * CurrentMMArch::PAGE_SIZE),
                    page_table_flags,
                )
            };
            if let Some(flusher) = result {
                flusher.flush();
            }
        }

        unsafe {
            core::intrinsics::copy_nonoverlapping(
                (buffer.as_ptr() as usize + offset as usize) as *const u8,
                vaddr as *mut u8,
                size as usize,
            );
        }

        if memsize > size {
            unsafe {
                core::slice::from_raw_parts_mut(
                    (vaddr as usize + size as usize) as *mut u8,
                    memsize as usize - size as usize,
                )
            }
            .fill(0);
        }
    }

    if let Some(interpreter_name) = elf_header.interpreter {
        let node = get_inode_by_path(interpreter_name.to_string()).ok_or(Errno::ENOENT)?;
        let interpreter_buffer = INTERPRETER_EHDR_START as *mut u8;

        let size = node.read().get_info().size;

        let aligned_vaddr = (interpreter_buffer as usize + CurrentMMArch::PAGE_SIZE - 1)
            & !(CurrentMMArch::PAGE_SIZE - 1);
        let count = (size as usize + CurrentMMArch::PAGE_SIZE - 1) / CurrentMMArch::PAGE_SIZE;

        for i in 0..count {
            let result = unsafe {
                mapper.map(
                    VirtualAddress::new(aligned_vaddr as usize + i * CurrentMMArch::PAGE_SIZE),
                    PageFlags::<CurrentMMArch>::new().write(true).user(true),
                )
            };
            if let Some(flusher) = result {
                flusher.flush();
            }
        }

        let interpreter_buffer =
            unsafe { core::slice::from_raw_parts_mut(interpreter_buffer, size) };

        node.read()
            .read_at(0, interpreter_buffer)
            .or(Err(Errno::E2BIG))?;

        let interpreter = Elf::parse(interpreter_buffer).or(Err(Errno::EINVAL))?;

        ldso_entry = INTERPRETER_LOAD_BASE + interpreter.entry as usize;

        for phdr in interpreter.program_headers.iter() {
            let vaddr = INTERPRETER_LOAD_BASE as u64 + phdr.p_vaddr;
            let offset = phdr.p_offset;
            let size = phdr.p_filesz;
            let memsize = phdr.p_memsz;

            if (vaddr as usize) < interpreter_load_start {
                interpreter_load_start = vaddr as usize;
            }
            if (vaddr as usize + memsize as usize) > interpreter_load_end {
                interpreter_load_end = vaddr as usize + memsize as usize;
            }

            if phdr.p_type != PT_LOAD {
                continue;
            }

            let page_table_flags = PageFlags::<CurrentMMArch>::new()
                .execute(true)
                .write(true)
                .user(true);

            let aligned_vaddr = (vaddr as usize) & !CurrentMMArch::PAGE_OFFSET_MASK;
            let aligned_end = (vaddr as usize + memsize as usize + CurrentMMArch::PAGE_SIZE - 1)
                & !(CurrentMMArch::PAGE_OFFSET_MASK);

            let count = (aligned_end - aligned_vaddr + CurrentMMArch::PAGE_SIZE - 1)
                / CurrentMMArch::PAGE_SIZE;

            for i in 0..count {
                let result = unsafe {
                    mapper.map(
                        VirtualAddress::new(aligned_vaddr as usize + i * CurrentMMArch::PAGE_SIZE),
                        page_table_flags,
                    )
                };
                if let Some(flusher) = result {
                    flusher.flush();
                }
            }

            unsafe {
                core::intrinsics::copy_nonoverlapping(
                    (interpreter_buffer.as_ptr() as usize + offset as usize) as *const u8,
                    vaddr as *mut u8,
                    size as usize,
                );
            }

            if memsize > size {
                unsafe {
                    core::slice::from_raw_parts_mut(
                        (vaddr as usize + size as usize) as *mut u8,
                        memsize as usize - size as usize,
                    )
                }
                .fill(0);
            }
        }

        use_ldso = true;
    }
    let count =
        (USER_STACK_SIZE as usize + CurrentMMArch::PAGE_SIZE - 1) / CurrentMMArch::PAGE_SIZE;

    for i in 0..count {
        let result = unsafe {
            mapper.map(
                VirtualAddress::new(
                    (USER_STACK_END - USER_STACK_SIZE) as usize + i * CurrentMMArch::PAGE_SIZE,
                ),
                PageFlags::new().write(true).user(true),
            )
        };
        if let Some(flusher) = result {
            flusher.flush();
        } else {
            return Err(Errno::ENOMEM);
        }
    }

    let mut proc_init_info = ProcInitInfo::new(unsafe { CStr::from_ptr(path).to_str().unwrap() });
    proc_init_info.args = args;
    proc_init_info.envs = envs;
    proc_init_info
        .auxv
        .insert(AtType::ENTRY as u8, elf_header.entry as usize);
    proc_init_info.auxv.insert(
        AtType::PHDR as u8,
        load_start + elf_header.header.e_phoff as usize,
    );
    proc_init_info
        .auxv
        .insert(AtType::PHNUM as u8, elf_header.header.e_phnum as usize);
    proc_init_info
        .auxv
        .insert(AtType::PHENT as u8, size_of::<ProgramHeader>());
    proc_init_info
        .auxv
        .insert(AtType::PAGESZ as u8, CurrentMMArch::PAGE_SIZE);
    proc_init_info.auxv.insert(
        AtType::BASE as u8,
        if use_ldso { INTERPRETER_LOAD_BASE } else { 0 },
    );

    get_current_context().write().memory_mappings.phdrs = elf_header.program_headers.clone();
    get_current_context()
        .write()
        .memory_mappings
        .interpreter_load_start = interpreter_load_start;
    get_current_context()
        .write()
        .memory_mappings
        .interpreter_load_end = interpreter_load_end;

    if let Ok((stack, _)) = unsafe { proc_init_info.push_at(USER_STACK_END) } {
        get_current_context().write().init_info = Some(proc_init_info);
        get_current_context().write().arch.go_to_user(
            if use_ldso {
                ldso_entry
            } else {
                elf_header.entry as usize
            },
            stack,
        );

        let stack_point = get_current_context().read().arch.address();
        unsafe { mapper.make_current() };
        unsafe { arch_to_user_mode(stack_point) };

        return Ok(());
    }

    Err(Errno::ENOMEM)
}

pub const EHDR_START: usize = 0x0000_1000_0000_0000;
pub const INTERPRETER_LOAD_BASE: usize = 0x0000_2000_0000_0000;
pub const INTERPRETER_EHDR_START: usize = 0x0000_3000_0000_0000;

pub fn sys_execve(
    path_ptr: *const core::ffi::c_char,
    argv: *const *mut core::ffi::c_char,
    envp: *const *mut core::ffi::c_char,
) -> Result<()> {
    arch_disable_intr();

    let path = unsafe { CStr::from_ptr(path_ptr) };

    let node = get_inode_by_path(path.to_str().or(Err(Errno::EINVAL))?.to_string())
        .ok_or(Errno::ENOENT)?;

    let buffer_start = EHDR_START;
    let size = node.read().get_info().size;

    let aligned_vaddr =
        (buffer_start as usize + CurrentMMArch::PAGE_SIZE - 1) & !(CurrentMMArch::PAGE_SIZE - 1);
    let count = (size as usize + CurrentMMArch::PAGE_SIZE - 1) / CurrentMMArch::PAGE_SIZE;

    let mut mapper = unsafe { PageMapper::current(rmm::TableKind::User, TheFrameAllocator) };

    for i in 0..count {
        let result = unsafe {
            mapper.map(
                VirtualAddress::new(aligned_vaddr as usize + i * CurrentMMArch::PAGE_SIZE),
                PageFlags::<CurrentMMArch>::new().write(true).user(true),
            )
        };
        if let Some(flusher) = result {
            flusher.flush();
        }
    }

    node.read().read_at(0, unsafe {
        core::slice::from_raw_parts_mut(buffer_start as *mut u8, size)
    })?;

    let buffer = unsafe {
        core::slice::from_raw_parts(buffer_start as *const u8, node.read().get_info().size)
    };

    execve_from_buffer(path_ptr, buffer, argv, envp)
}
