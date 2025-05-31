use core::fmt::{self, Write};

use alloc::boxed::Box;
use os_terminal::{Terminal, font::BitmapFont};
use spin::{Lazy, Mutex};

use crate::drivers::display::Display;

#[macro_export]
macro_rules! print {
    ($($arg: tt)*) => ($crate::drivers::term::_print(format_args!($($arg)*)));
}

#[macro_export]
macro_rules! println {
    () => ($crate::print!("\n"));
    ($($arg: tt)*) => ($crate::print!("{}\n", format_args!($($arg)*)));
}

#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    TERMINAL.lock().write_fmt(args).unwrap();
}

pub static TERMINAL: Lazy<Mutex<Terminal<Display>>> = Lazy::new(|| {
    let mut terminal = Terminal::new(Display::default());
    terminal.set_font_manager(Box::new(BitmapFont));
    Mutex::new(terminal)
});
