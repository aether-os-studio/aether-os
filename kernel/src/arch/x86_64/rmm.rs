use crate::memory::frame::TheFrameAllocator;

pub type PageMapper = rmm::PageMapper<rmm::X8664Arch, TheFrameAllocator>;
