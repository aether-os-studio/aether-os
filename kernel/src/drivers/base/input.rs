use crossbeam_queue::ArrayQueue;
use spin::Lazy;

pub static BYTES: Lazy<ArrayQueue<u8>> = Lazy::new(|| ArrayQueue::new(2048));
