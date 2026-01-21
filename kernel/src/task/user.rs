use crate::{
    arch::CurrentRmmArch,
    fs::vfs::file::ArcFile,
    init::memory::{FRAME_ALLOCATOR, KERNEL_PAGE_TABLE_PHYS, PAGE_SIZE, align_down, align_up},
    task::{ArcTask, Task, add_task},
};
use alloc::{string::String, vec, vec::Vec};
use core::mem::size_of;
use rmm::{
    Arch, FrameAllocator, PageFlags, PageMapper, PhysicalAddress, TableKind, VirtualAddress,
};

const USER_STACK_TOP: usize = 0x0000_7000_0000_0000;
const USER_STACK_SIZE: usize = 8 * 1024 * 1024;
const INTERP_LOAD_BASE: usize = 0x0000_7fff_0000_0000;
const PIE_LOAD_BASE: usize = 0x0000_7fff_ff00_0000;

type PageFlagsType = PageFlags<CurrentRmmArch>;

fn make_page_flags(_read: bool, write: bool, exec: bool, user: bool) -> PageFlagsType {
    PageFlags::new().write(write).execute(exec).user(user)
}

fn create_user_page_table() -> PhysicalAddress {
    let new_page_table = unsafe { FRAME_ALLOCATOR.lock().allocate_one() }
        .expect("Failed to allocate new page table");
    let kernel_page_table =
        PhysicalAddress::new(KERNEL_PAGE_TABLE_PHYS.load(core::sync::atomic::Ordering::SeqCst));
    let new_page_table_virt = unsafe { CurrentRmmArch::phys_to_virt(new_page_table) };
    let kernel_page_table_virt = unsafe { CurrentRmmArch::phys_to_virt(kernel_page_table) };
    #[cfg(any(target_arch = "x86_64", target_arch = "riscv64"))]
    unsafe {
        core::ptr::copy_nonoverlapping(
            kernel_page_table_virt.add(PAGE_SIZE / 2).data() as *const u8,
            new_page_table_virt.add(PAGE_SIZE / 2).data() as *mut u8,
            PAGE_SIZE / 2,
        );
        core::ptr::write_bytes(new_page_table_virt.data() as *mut u8, 0, PAGE_SIZE / 2);
    };
    #[cfg(not(any(target_arch = "x86_64", target_arch = "riscv64")))]
    unsafe {
        core::ptr::write_bytes(new_page_table_virt.data() as *mut u8, 0, PAGE_SIZE);
    };
    new_page_table
}

fn get_random_bytes() -> [u8; 16] {
    [0u8; 16]
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Elf64Header {
    pub e_ident: [u8; 16],
    pub e_type: u16,
    pub e_machine: u16,
    pub e_version: u32,
    pub e_entry: u64,
    pub e_phoff: u64,
    pub e_shoff: u64,
    pub e_flags: u32,
    pub e_ehsize: u16,
    pub e_phentsize: u16,
    pub e_phnum: u16,
    pub e_shentsize: u16,
    pub e_shnum: u16,
    pub e_shstrndx: u16,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Elf64ProgramHeader {
    pub p_type: u32,
    pub p_flags: u32,
    pub p_offset: u64,
    pub p_vaddr: u64,
    pub p_paddr: u64,
    pub p_filesz: u64,
    pub p_memsz: u64,
    pub p_align: u64,
}

pub const ELF_MAGIC: [u8; 4] = [0x7f, b'E', b'L', b'F'];
pub const ELFCLASS64: u8 = 2;
pub const ET_EXEC: u16 = 2;
pub const ET_DYN: u16 = 3;
pub const PT_NULL: u32 = 0;
pub const PT_LOAD: u32 = 1;
pub const PT_INTERP: u32 = 3;
pub const PT_PHDR: u32 = 6;

pub const PF_X: u32 = 1;
pub const PF_W: u32 = 2;
pub const PF_R: u32 = 4;

pub const AT_NULL: usize = 0;
pub const AT_IGNORE: usize = 1;
pub const AT_EXECFD: usize = 2;
pub const AT_PHDR: usize = 3;
pub const AT_PHENT: usize = 4;
pub const AT_PHNUM: usize = 5;
pub const AT_PAGESZ: usize = 6;
pub const AT_BASE: usize = 7;
pub const AT_FLAGS: usize = 8;
pub const AT_ENTRY: usize = 9;
pub const AT_UID: usize = 11;
pub const AT_EUID: usize = 12;
pub const AT_GID: usize = 13;
pub const AT_EGID: usize = 14;
pub const AT_PLATFORM: usize = 15;
pub const AT_HWCAP: usize = 16;
pub const AT_CLKTCK: usize = 17;
pub const AT_SECURE: usize = 23;
pub const AT_RANDOM: usize = 25;
pub const AT_HWCAP2: usize = 26;
pub const AT_EXECFN: usize = 31;

struct ElfLoadResult {
    entry_point: usize,
    phdr_vaddr: usize,
    phent_size: usize,
    phnum: usize,
    interp_path: Option<String>,
}

pub fn create_user_task(
    name: String,
    file: ArcFile,
    argv: Vec<String>,
    envp: Vec<String>,
) -> Result<ArcTask, &'static str> {
    let user_page_table = create_user_page_table();

    let mut frame_allocator = FRAME_ALLOCATOR.lock();
    let mut page_mapper = unsafe {
        PageMapper::<CurrentRmmArch, _>::new(
            TableKind::User,
            user_page_table,
            &mut *frame_allocator,
        )
    };

    let elf_result = load_elf(&file, &mut page_mapper, 0)?;

    let (entry_point, interp_base) = if let Some(interp_path) = &elf_result.interp_path {
        let interp_file = {
            let mut file_guard = file.write();
            file_guard
                .lookup(interp_path.clone(), true)
                .ok_or("Failed to find interpreter")?
        };

        let interp_result = load_elf(&interp_file, &mut page_mapper, INTERP_LOAD_BASE)?;
        (interp_result.entry_point, INTERP_LOAD_BASE)
    } else {
        (elf_result.entry_point, 0)
    };

    let stack_bottom = USER_STACK_TOP - USER_STACK_SIZE;
    allocate_user_stack(&mut page_mapper, stack_bottom, USER_STACK_SIZE)?;

    let stack_pointer = setup_user_stack(
        &mut page_mapper,
        USER_STACK_TOP,
        &argv,
        &envp,
        &elf_result,
        interp_base,
    )?;

    drop(frame_allocator);

    let task = Task::new(name);

    task.write().page_table_addr = user_page_table;

    {
        let mut task_guard = task.write();
        task_guard.set_user_context_info(entry_point, VirtualAddress::new(stack_pointer), None);
    }

    add_task(task.clone());

    Ok(task)
}

fn load_elf<A: FrameAllocator>(
    file: &ArcFile,
    page_mapper: &mut PageMapper<CurrentRmmArch, A>,
    load_base: usize,
) -> Result<ElfLoadResult, &'static str> {
    let mut header_buf = [0u8; size_of::<Elf64Header>()];
    {
        let mut file_guard = file.write();
        file_guard
            .read(&mut header_buf, 0)
            .ok_or("Failed to read ELF header")?;
    }

    let header: Elf64Header = unsafe { core::ptr::read(header_buf.as_ptr() as *const _) };

    if header.e_ident[0..4] != ELF_MAGIC {
        return Err("Invalid ELF magic");
    }
    if header.e_ident[4] != ELFCLASS64 {
        return Err("Not a 64-bit ELF");
    }
    if header.e_type != ET_EXEC && header.e_type != ET_DYN {
        return Err("Not an executable or shared object");
    }

    let actual_load_base = if header.e_type == ET_DYN && load_base == 0 {
        PIE_LOAD_BASE
    } else {
        load_base
    };

    let ph_size = header.e_phentsize as usize * header.e_phnum as usize;
    let mut ph_buf = vec![0u8; ph_size];
    {
        let mut file_guard = file.write();
        file_guard
            .read(&mut ph_buf, header.e_phoff as usize)
            .ok_or("Failed to read program headers")?;
    }

    let mut interp_path = None;
    let mut phdr_vaddr = 0usize;

    for i in 0..header.e_phnum as usize {
        let ph_offset = i * header.e_phentsize as usize;
        let ph: Elf64ProgramHeader =
            unsafe { core::ptr::read(ph_buf[ph_offset..].as_ptr() as *const _) };

        match ph.p_type {
            PT_LOAD => {
                load_segment(file, page_mapper, &ph, actual_load_base)?;
            }
            PT_INTERP => {
                let mut interp_buf = vec![0u8; ph.p_filesz as usize];
                {
                    let mut file_guard = file.write();
                    file_guard
                        .read(&mut interp_buf, ph.p_offset as usize)
                        .ok_or("Failed to read interpreter path")?;
                }
                if let Some(pos) = interp_buf.iter().position(|&b| b == 0) {
                    interp_buf.truncate(pos);
                }
                interp_path =
                    Some(String::from_utf8(interp_buf).map_err(|_| "Invalid interpreter path")?);
            }
            PT_PHDR => {
                phdr_vaddr = actual_load_base + ph.p_vaddr as usize;
            }
            _ => {}
        }
    }

    if phdr_vaddr == 0 {
        phdr_vaddr = actual_load_base + header.e_phoff as usize;
    }

    Ok(ElfLoadResult {
        entry_point: actual_load_base + header.e_entry as usize,
        phdr_vaddr,
        phent_size: header.e_phentsize as usize,
        phnum: header.e_phnum as usize,
        interp_path,
    })
}

fn load_segment<A: FrameAllocator>(
    file: &ArcFile,
    page_mapper: &mut PageMapper<CurrentRmmArch, A>,
    ph: &Elf64ProgramHeader,
    load_base: usize,
) -> Result<(), &'static str> {
    if ph.p_memsz == 0 {
        return Ok(());
    }

    let vaddr_start = load_base + ph.p_vaddr as usize;
    let vaddr_end = vaddr_start + ph.p_memsz as usize;

    let page_start = align_down(vaddr_start);
    let page_end = align_up(vaddr_end);

    let flags = make_page_flags(
        ph.p_flags & PF_R != 0,
        ph.p_flags & PF_W != 0,
        ph.p_flags & PF_X != 0,
        true,
    );

    for vaddr in (page_start..page_end).step_by(PAGE_SIZE) {
        unsafe {
            page_mapper
                .map(VirtualAddress::new(vaddr), flags)
                .ok_or("Failed to map ELF segment")?
                .ignore();
        }
    }

    if ph.p_filesz > 0 {
        let mut buf = vec![0u8; ph.p_filesz as usize];
        {
            let mut file_guard = file.write();
            file_guard
                .read(&mut buf, ph.p_offset as usize)
                .ok_or("Failed to read segment data")?;
        }

        write_to_user_space(page_mapper, vaddr_start, &buf)?;
    }
    if ph.p_memsz > ph.p_filesz {
        let buf = vec![0u8; (ph.p_memsz - ph.p_filesz) as usize];
        write_to_user_space(page_mapper, vaddr_start + ph.p_filesz as usize, &buf)?;
    }

    Ok(())
}

fn allocate_user_stack<A: FrameAllocator>(
    page_mapper: &mut PageMapper<CurrentRmmArch, A>,
    stack_bottom: usize,
    stack_size: usize,
) -> Result<(), &'static str> {
    let flags = make_page_flags(true, true, false, true);

    for offset in (0..stack_size).step_by(PAGE_SIZE) {
        let vaddr = stack_bottom + offset;
        unsafe {
            page_mapper
                .map(VirtualAddress::new(vaddr), flags)
                .ok_or("Failed to map stack page")?
                .ignore();
        }
    }

    Ok(())
}

fn setup_user_stack<A: FrameAllocator>(
    page_mapper: &mut PageMapper<CurrentRmmArch, A>,
    stack_top: usize,
    argv: &[String],
    envp: &[String],
    elf_result: &ElfLoadResult,
    interp_base: usize,
) -> Result<usize, &'static str> {
    let mut sp = stack_top;

    let random_bytes = get_random_bytes();
    sp -= 16;
    let at_random_addr = sp;
    write_to_user_space(page_mapper, sp, &random_bytes)?;

    #[cfg(target_arch = "x86_64")]
    let platform = b"x86_64\0";
    #[cfg(target_arch = "aarch64")]
    let platform = b"aarch64\0";
    #[cfg(target_arch = "riscv64")]
    let platform = b"riscv64\0";
    #[cfg(target_arch = "loongarch64")]
    let platform = b"loongarch64\0";

    sp -= platform.len();
    let platform_addr = sp;
    write_to_user_space(page_mapper, sp, platform)?;

    let execfn = if !argv.is_empty() {
        argv[0].as_bytes()
    } else {
        b"unknown"
    };
    sp -= execfn.len() + 1;
    let execfn_addr = sp;
    write_to_user_space(page_mapper, sp, execfn)?;
    write_to_user_space(page_mapper, sp + execfn.len(), &[0u8])?;

    let mut envp_addrs = Vec::with_capacity(envp.len());
    for env in envp.iter().rev() {
        sp -= env.len() + 1;
        envp_addrs.push(sp);
        write_to_user_space(page_mapper, sp, env.as_bytes())?;
        write_to_user_space(page_mapper, sp + env.len(), &[0u8])?;
    }
    envp_addrs.reverse();

    let mut argv_addrs = Vec::with_capacity(argv.len());
    for arg in argv.iter().rev() {
        sp -= arg.len() + 1;
        argv_addrs.push(sp);
        write_to_user_space(page_mapper, sp, arg.as_bytes())?;
        write_to_user_space(page_mapper, sp + arg.len(), &[0u8])?;
    }
    argv_addrs.reverse();

    sp &= !0xF;

    let auxv: Vec<(usize, usize)> = vec![
        (AT_PHDR, elf_result.phdr_vaddr),
        (AT_PHENT, elf_result.phent_size),
        (AT_PHNUM, elf_result.phnum),
        (AT_PAGESZ, PAGE_SIZE),
        (AT_BASE, interp_base),
        (AT_ENTRY, elf_result.entry_point),
        (AT_UID, 0),
        (AT_EUID, 0),
        (AT_GID, 0),
        (AT_EGID, 0),
        (AT_SECURE, 0),
        (AT_RANDOM, at_random_addr),
        (AT_PLATFORM, platform_addr),
        (AT_EXECFN, execfn_addr),
        (AT_NULL, 0),
    ];

    let argc = argv.len();
    let auxv_size = auxv.len() * 2 * size_of::<usize>();
    let envp_ptr_size = (envp.len() + 1) * size_of::<usize>();
    let argv_ptr_size = (argc + 1) * size_of::<usize>();
    let argc_size = size_of::<usize>();
    let total_size = auxv_size + envp_ptr_size + argv_ptr_size + argc_size;

    sp -= total_size;
    sp &= !0xF;

    let stack_pointer = sp;

    write_usize(page_mapper, sp, argc)?;
    sp += size_of::<usize>();

    for &addr in &argv_addrs {
        write_usize(page_mapper, sp, addr)?;
        sp += size_of::<usize>();
    }
    write_usize(page_mapper, sp, 0)?;
    sp += size_of::<usize>();

    for &addr in &envp_addrs {
        write_usize(page_mapper, sp, addr)?;
        sp += size_of::<usize>();
    }
    write_usize(page_mapper, sp, 0)?;
    sp += size_of::<usize>();

    for (aux_type, aux_val) in auxv {
        write_usize(page_mapper, sp, aux_type)?;
        sp += size_of::<usize>();
        write_usize(page_mapper, sp, aux_val)?;
        sp += size_of::<usize>();
    }

    Ok(stack_pointer)
}

fn write_to_user_space<A: FrameAllocator>(
    page_mapper: &mut PageMapper<CurrentRmmArch, A>,
    vaddr: usize,
    data: &[u8],
) -> Result<(), &'static str> {
    let mut offset = 0;
    while offset < data.len() {
        let page_vaddr = (vaddr + offset) & !(PAGE_SIZE - 1);
        let page_offset = (vaddr + offset) % PAGE_SIZE;
        let bytes_in_page = core::cmp::min(PAGE_SIZE - page_offset, data.len() - offset);

        let (phys, _flags) = page_mapper
            .translate(VirtualAddress::new(page_vaddr))
            .ok_or("Page not mapped for writing")?;

        let kernel_vaddr = unsafe { CurrentRmmArch::phys_to_virt(phys) };

        unsafe {
            core::ptr::copy_nonoverlapping(
                data[offset..].as_ptr(),
                (kernel_vaddr.data() + page_offset) as *mut u8,
                bytes_in_page,
            );
        }

        offset += bytes_in_page;
    }
    Ok(())
}

fn write_usize<A: FrameAllocator>(
    page_mapper: &mut PageMapper<CurrentRmmArch, A>,
    vaddr: usize,
    value: usize,
) -> Result<(), &'static str> {
    let bytes = value.to_ne_bytes();
    write_to_user_space(page_mapper, vaddr, &bytes)
}
