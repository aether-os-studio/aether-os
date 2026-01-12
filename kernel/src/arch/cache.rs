pub trait CacheArch {
    fn clean_range(_addr: u64, _size: usize) {}
    fn invalidate_range(_addr: u64, _size: usize) {}
    fn flush_range(_addr: u64, _size: usize) {}
}
