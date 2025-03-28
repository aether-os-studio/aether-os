use alloc::boxed::Box;
use alloc::string::String;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};
use crossbeam_queue::ArrayQueue;
use os_terminal::font::BitmapFont;
use os_terminal::{MouseInput, Terminal};
use pc_keyboard::{DecodedKey, HandleControl, KeyCode, Keyboard, ScancodeSet1, layouts};
use spin::Lazy;

use super::writer::TerminalWriter;
use crate::kdevice::display::Display;
use crate::kdevice::mouse::{MOUSE_BUFFER, MouseEvent};
use crate::syscall::fs::KEYCODE_QUEUE;
use crate::syscall::sys_yield;

const SCANCODE_QUEUE_SIZE: usize = 128;
const TERMINAL_BUFFER_SIZE: usize = 1024;

pub static SCANCODE_QUEUE: Lazy<ArrayQueue<u8>> =
    Lazy::new(|| ArrayQueue::new(SCANCODE_QUEUE_SIZE));
pub static TERMINAL_BUFFER: Lazy<ArrayQueue<String>> =
    Lazy::new(|| ArrayQueue::new(TERMINAL_BUFFER_SIZE));

static NEED_FLUSH: AtomicBool = AtomicBool::new(false);

fn terminal_flush(terminal: &mut Terminal<Display>) {
    while let Some(s) = TERMINAL_BUFFER.pop() {
        let _ = terminal.write_str(&s);
        NEED_FLUSH.store(true, Ordering::Relaxed);
    }

    if NEED_FLUSH.swap(false, Ordering::Relaxed) {
        terminal.flush();
    }
}

fn terminal_event(
    terminal: &mut Terminal<Display>,
    keyboard: &mut Keyboard<layouts::Us104Key, ScancodeSet1>,
) {
    while let Some(scancode) = SCANCODE_QUEUE.pop() {
        terminal.handle_keyboard(scancode);
        if let Ok(Some(key_event)) = keyboard.add_byte(scancode) {
            if let Some(key) = keyboard.process_keyevent(key_event) {
                match key {
                    DecodedKey::RawKey(raw_key) => match raw_key {
                        KeyCode::Backspace => KEYCODE_QUEUE.push(8).unwrap(),
                        KeyCode::Oem7 => KEYCODE_QUEUE.push(b'\\').unwrap(),
                        _ => {}
                    },
                    DecodedKey::Unicode(ch) => KEYCODE_QUEUE.push(ch as u8).unwrap(),
                }
            }
        }
        NEED_FLUSH.store(true, Ordering::Relaxed);
    }

    while let Some(MouseEvent::Scroll(delta)) = MOUSE_BUFFER.pop() {
        terminal.handle_mouse(MouseInput::Scroll(delta));
        NEED_FLUSH.store(true, Ordering::Relaxed);
    }
}

pub fn terminal_thread() {
    let mut terminal = Terminal::new(Display::default());
    terminal.set_auto_crnl(true);
    terminal.set_auto_flush(false);
    terminal.set_scroll_speed(5);
    terminal.set_font_manager(Box::new(BitmapFont));

    terminal.set_pty_writer(Box::new(|s: String| TerminalWriter.write_str(&s).unwrap()));

    let mut keyboard = Keyboard::new(
        ScancodeSet1::new(),
        layouts::Us104Key,
        HandleControl::Ignore,
    );

    loop {
        terminal_event(&mut terminal, &mut keyboard);
        terminal_flush(&mut terminal);
        sys_yield();
    }
}
