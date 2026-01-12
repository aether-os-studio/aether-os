pub trait TimeArch {
    fn nano_time() -> u64;
    fn delay(ns: u64);
}
