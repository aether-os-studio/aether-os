use core::sync::atomic::AtomicUsize;

use alloc::{
    collections::vec_deque::VecDeque,
    string::{String, ToString},
    sync::Arc,
};
use object::{Error, File, Object, ObjectSegment};
use spin::{Mutex, RwLock};
use x86_64::{VirtAddr, structures::paging::OffsetPageTable};

use crate::{
    memory::{ExtendedPageTable, KERNEL_PAGE_TABLE, MappingType, MemoryManager},
    pctable::gdt::Selectors,
};

use super::{
    ctx::Context,
    kstack::{KernelStack, UserStack},
};

pub static TASKS: Mutex<VecDeque<Arc<RwLock<Task>>>> = Mutex::new(VecDeque::new());

#[allow(dead_code)]
pub struct Task {
    pub id: usize,
    pub name: String,
    pub page_table: OffsetPageTable<'static>,
    pub kernel_stack: KernelStack,
    pub context: Context,
    pub sleeping: bool,
}

impl Task {
    pub fn new(name: &'static str, page_table: OffsetPageTable<'static>) -> Arc<RwLock<Task>> {
        static ID: AtomicUsize = AtomicUsize::new(1);

        let this = Task {
            id: ID.fetch_add(1, core::sync::atomic::Ordering::SeqCst),
            name: name.to_string(),
            context: Context::default(),
            kernel_stack: KernelStack::new(),
            sleeping: false,
            page_table,
        };

        Arc::new(RwLock::new(this))
    }

    pub fn remove_self(&self) {
        let mut tasks = TASKS.lock();
        if let Some(index) = tasks
            .iter()
            .position(|process| process.read().id == self.id)
        {
            tasks.remove(index);
        }
    }

    pub fn create(
        name: &'static str,
        _argv: usize,
        _envp: usize,
        data: &'static [u8],
    ) -> Result<(), Error> {
        let elf_file = File::parse(data)?;
        let mut page_table = unsafe { KERNEL_PAGE_TABLE.lock().deep_copy() };
        let page_table_phys_addr = page_table.physical_address();

        for segment in elf_file.segments() {
            let address = VirtAddr::new(segment.address());

            MemoryManager::alloc_range(
                address,
                segment.size(),
                MappingType::UserCode.flags(),
                &mut page_table,
            )
            .expect("Failed to allocate memory for ELF segment");

            if let Ok(data) = segment.data() {
                page_table.write_to_mapped_address(data, address);
            }
        }

        UserStack::map(&mut page_table);

        let this = Self::new(name, page_table);

        let mut this = this.write();
        this.context.init(
            elf_file.entry() as usize,
            UserStack::end_address(),
            page_table_phys_addr,
            Selectors::get_user_segments(),
            // Not supported
            0,
            0,
            0,
        );

        Ok(())
    }
}
