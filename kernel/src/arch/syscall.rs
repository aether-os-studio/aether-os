pub trait SyscallArch {
    unsafe fn copy_from_user(dst: usize, src: usize, len: usize) -> usize;
    unsafe fn copy_to_user(dst: usize, src: usize, len: usize) -> usize;
}
