use core::sync::atomic::{AtomicBool, Ordering};

use crate::fs::vfs::IndexNodeInfo;

use super::{IndexNode, IndexNodeType};
use alloc::sync::Arc;
use crossbeam_queue::ArrayQueue;
use spin::RwLock;

pub struct Pipe {
    buffer: ArrayQueue<u8>,
    read_open: AtomicBool,
    write_open: AtomicBool,
}

#[repr(C)]
pub struct PipeIndexNode {
    pipe: Arc<RwLock<Pipe>>,
    is_write: bool,
}

impl PipeIndexNode {
    pub fn new_pair() -> (Self, Self) {
        let pipe = Arc::new(RwLock::new(Pipe {
            buffer: ArrayQueue::new(4096),
            read_open: AtomicBool::new(true),
            write_open: AtomicBool::new(true),
        }));

        (
            Self {
                pipe: pipe.clone(),
                is_write: false,
            },
            Self {
                pipe,
                is_write: true,
            },
        )
    }
}

impl IndexNode for PipeIndexNode {
    fn when_mounted(&mut self, path: alloc::string::String, father: Option<super::IndexNodeRef>) {}

    fn when_umounted(&mut self) {}

    fn read_at(&self, _offset: usize, buf: &mut [u8]) -> crate::syscall::Result<usize> {
        if self.is_write {
            return Err(crate::errno::Errno::EPERM);
        }

        let pipe = self.pipe.read();
        let mut bytes_read = 0;

        while bytes_read < buf.len() {
            if let Some(byte) = pipe.buffer.pop() {
                buf[bytes_read] = byte;
                bytes_read += 1;
            } else {
                if !pipe.write_open.load(Ordering::Relaxed) {
                    break;
                }
                core::hint::spin_loop();
            }
        }

        Ok(bytes_read)
    }

    fn write_at(&self, _offset: usize, buf: &[u8]) -> crate::syscall::Result<usize> {
        if !self.is_write {
            return Err(crate::errno::Errno::EPERM);
        }

        let pipe = self.pipe.read();
        for &byte in buf {
            while pipe.buffer.push(byte).is_err() {
                if !pipe.read_open.load(Ordering::Relaxed) {
                    return Err(crate::errno::Errno::EPIPE);
                }
                core::hint::spin_loop();
            }
        }
        Ok(buf.len())
    }

    fn get_info(&self) -> IndexNodeInfo {
        let mut info = IndexNodeInfo::default();
        info.inode_type = IndexNodeType::Dev;
        info
    }
}
