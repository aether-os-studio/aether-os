use core::fmt::Write;

use alloc::boxed::Box;
use os_terminal::{Terminal, font::BitmapFont};
use spin::{Lazy, Mutex};

use super::display::Display;

pub static TERMINAL: Lazy<Mutex<Terminal<Display>>> = Lazy::new(|| {
    let mut terminal = Terminal::new(Display::default());
    terminal.set_font_manager(Box::new(BitmapFont));
    Mutex::new(terminal)
});

pub fn _print(args: core::fmt::Arguments) {
    let _ = TERMINAL.lock().write_fmt(args);
}

#[macro_export]
macro_rules! print {
        ($($arg:tt)*) => (
            $crate::kdevice::terminal::_print(format_args!($($arg)*))
        )
    }

#[macro_export]
macro_rules! println {
        () => ($crate::print!("\n"));
        ($($arg:tt)*) => ($crate::print!("{}\n", format_args!($($arg)*)))
    }
