use core::hint::spin_loop;

use alloc::{string::ToString, sync::Arc};
use crossbeam_queue::ArrayQueue;
use pc_keyboard::{DecodedKey, KeyCode, Keyboard, ScancodeSet1, layouts::Us104Key};
use spin::{Lazy, RwLock};

use crate::{
    arch::arch_enable_intr,
    drivers::base::input::BYTES,
    proc::{Context, sched::SCHEDULER},
};

pub static SCANCODES: Lazy<ArrayQueue<u8>> = Lazy::new(|| ArrayQueue::new(2048));

fn ps2_keyboard_thread() -> ! {
    let mut keyboard = Keyboard::new(
        ScancodeSet1::new(),
        Us104Key,
        pc_keyboard::HandleControl::Ignore,
    );

    loop {
        if let Some(scan_code) = SCANCODES.pop() {
            if let Ok(Some(key_event)) = keyboard.add_byte(scan_code) {
                if let Some(key) = keyboard.process_keyevent(key_event) {
                    match key {
                        DecodedKey::RawKey(raw_key) => match raw_key {
                            KeyCode::Backspace => BYTES.push(8 as u8).unwrap(),
                            KeyCode::Oem7 => BYTES.push(b'\\').unwrap(),
                            _ => {}
                        },
                        DecodedKey::Unicode(ch) => BYTES.push(ch as u8).unwrap(),
                    }
                }
            }
        }

        arch_enable_intr();

        spin_loop();
    }
}

pub fn init() {
    SCHEDULER.lock().add(Arc::new(RwLock::new(Context::new(
        "ps2 keyboard".to_string(),
        ps2_keyboard_thread as usize,
    ))));
}
