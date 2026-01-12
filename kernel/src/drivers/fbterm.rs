use core::{
    fmt::{self, Write},
    sync::atomic::AtomicBool,
};

use alloc::boxed::Box;
use os_terminal::{DrawTarget, Rgb, Terminal, font::BitmapFont};
use spin::{Lazy, Mutex};

use crate::drivers::framebuffer::FRAME_BUFFERS;

pub struct Display {
    width: usize,
    height: usize,
    stride: usize,
    buffer: *mut u32,
    shifts: (u8, u8, u8),
}

impl DrawTarget for Display {
    fn size(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    #[inline(always)]
    fn draw_pixel(&mut self, x: usize, y: usize, color: u32) {
        unsafe { self.buffer.add(y * self.stride + x).write(color) }
    }

    #[inline(always)]
    fn rgb_to_pixel(&self, rgb: Rgb) -> u32 {
        ((rgb.0 as u32) << self.shifts.0)
            | ((rgb.1 as u32) << self.shifts.1)
            | ((rgb.2 as u32) << self.shifts.2)
    }
}

impl Default for Display {
    fn default() -> Self {
        let frame_buffers = FRAME_BUFFERS.lock();
        let frame_buffer = frame_buffers.iter().next().unwrap();

        let shifts = (
            frame_buffer.red_mask_shift as u8,
            frame_buffer.green_mask_shift as u8,
            frame_buffer.blue_mask_shift as u8,
        );

        Self {
            shifts,
            width: frame_buffer.width,
            height: frame_buffer.height,
            buffer: frame_buffer.address as *mut u32,
            stride: frame_buffer.pitch / size_of::<u32>(),
        }
    }
}

unsafe impl Send for Display {}

pub static TERMINAL: Lazy<Mutex<Terminal<Display>>> = Lazy::new(|| {
    let mut terminal = Terminal::new(Display::default());
    terminal.set_auto_flush(true);
    terminal.set_crnl_mapping(true);
    terminal.set_font_manager(Box::new(BitmapFont));
    Mutex::new(terminal)
});

pub static TERMINAL_INITIALIZED: AtomicBool = AtomicBool::new(false);

pub struct TerminalWriter;

impl fmt::Write for TerminalWriter {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        TERMINAL.lock().process(s.as_bytes());
        Ok(())
    }
}

pub fn _print(args: fmt::Arguments) {
    if TERMINAL_INITIALIZED.load(core::sync::atomic::Ordering::SeqCst) {
        let _ = TerminalWriter.write_fmt(args);
    }
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => (
        $crate::drivers::fbterm::_print(format_args!($($arg)*))
    )
}

#[macro_export]
macro_rules! println {
    () => ($crate::print!("\n"));
    ($($arg:tt)*) => ($crate::print!("{}\n", format_args!($($arg)*)))
}

pub fn init() {
    TERMINAL_INITIALIZED.store(true, core::sync::atomic::Ordering::SeqCst);
}
