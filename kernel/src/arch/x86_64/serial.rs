use core::fmt::{self, Write};
use spin::{Lazy, Mutex};
use uart_16550::SerialPort;

#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    SERIAL.lock().write_fmt(args).unwrap();
}

#[macro_export]
macro_rules! serial_print {
    ($($arg:tt)*) => (
        $crate::arch::serial::_print(format_args!($($arg)*))
    );
}

#[macro_export]
macro_rules! serial_println {
    () => ($crate::serial_print!("\n"));
    ($($arg:tt)*) => ($crate::serial_print!("{}\n", format_args!($($arg)*)));
}

pub static SERIAL: Lazy<Mutex<SerialPort>> = Lazy::new(|| {
    let mut serial_port = unsafe { SerialPort::new(0x3f8) };
    serial_port.init();
    Mutex::new(serial_port)
});
