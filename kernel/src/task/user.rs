// use alloc::{collections::btree_map::BTreeMap, string::String, vec::Vec};
// use goblin::elf::Elf;
// use rmm::{FrameAllocator, PageFlags, PageMapper, PhysicalAddress, TableKind, VirtualAddress};

// use crate::{
//     arch::CurrentRmmArch,
//     fs::{ROOT_DIR, vfs::file::ArcFile},
//     init::memory::{FRAME_ALLOCATOR, align_down},
//     task::{ArcTask, Task},
//     utils::Dma,
// };

// #[derive(Debug, Clone, Copy)]
// pub struct ExecResult {
//     entry: VirtualAddress,
//     stack: VirtualAddress,
// }

// impl ExecResult {
//     pub fn new(entry: VirtualAddress, stack: VirtualAddress) -> Self {
//         Self { entry, stack }
//     }

//     pub fn entry(&self) -> VirtualAddress {
//         self.entry
//     }

//     pub fn stack(&self) -> VirtualAddress {
//         self.stack
//     }
// }

// fn do_execve(
//     file: ArcFile,
//     page_table: PhysicalAddress,
//     args: Option<Vec<String>>,
//     envs: Option<Vec<String>>,
// ) -> Option<ExecResult> {
//     let mut auxv = BTreeMap::new();

//     let file_size = file.read().size()?;
//     let mut buffer = vec![0u8; file_size];
//     file.write().read(&mut buffer, 0);

//     let elf = Elf::parse(&buffer).ok()?;

//     let mut frame_allocator = FRAME_ALLOCATOR.lock();
//     let mut page_mapper =
//         unsafe { PageMapper::new(TableKind::User, page_table, &mut *frame_allocator) };

//     for phdr in elf.program_headers.iter() {
//         let vaddr = phdr.p_vaddr as usize;
//         let aligned_vaddr = align_down(vaddr);
//         let aligned_vaddr = VirtualAddress::new(aligned_vaddr);
//         if let Some(flusher) = unsafe {
//             page_mapper.map(
//                 aligned_vaddr,
//                 PageFlags::<CurrentRmmArch>::new()
//                     .write(phdr.is_write())
//                     .execute(phdr.is_executable()),
//             )
//         } {
//             flusher.flush();
//         }
//     }

//     None
// }

// pub fn create_user_task(path: String, args: Vec<String>) -> Option<ArcTask> {
//     let task_file = ROOT_DIR.write().lookup(path.clone(), true)?;

//     let new_page_table = unsafe { FRAME_ALLOCATOR.lock().allocate_one() }.unwrap();

//     let exec_result = do_execve(task_file.clone(), new_page_table, Some(args), None)?;

//     let task = Task::new(path);
//     task.write()
//         .set_user_context_info(exec_result.entry().data(), exec_result.stack(), None);

//     Some(task)
// }
