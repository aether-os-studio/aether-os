//! Utilities for tests in the whole crate.

use alloc::string::String;
use std::format;
use std::fs::File;
use std::io::copy;
use std::process::Command;
use std::sync::atomic::{AtomicU32, Ordering};

use spin::Lazy;

use crate::error::Error;

/// Common `LOREM IPSUM` string used for tests.
pub const LOREM: &str = include_str!("../tests/lorem.txt");

/// Length of [`LOREM`].
pub const LOREM_LENGTH: usize = LOREM.len();

/// Stores the next unique device id returned by [`new_device_id`].
static DEVICE_ID: Lazy<AtomicU32> = Lazy::new(AtomicU32::default);

/// Returns a new unique device ID (useful for tests).
pub fn new_device_id() -> u32 {
    DEVICE_ID.fetch_add(1, Ordering::Relaxed)
}

/// Stores the next unique file id returned by [`new_file_id`].
static FILE_ID: Lazy<AtomicU32> = Lazy::new(AtomicU32::default);

/// Returns a new unique file ID (useful for tests).
pub fn new_file_id() -> u32 {
    FILE_ID.fetch_add(1, Ordering::Relaxed)
}

/// Copies the file at the given path and returns a temporary file with the same content.
///
/// # Errors
///
/// Returns a [`Error::IO`] error if the given file could not be opened or copied to a temporary file.
pub fn copy_file(path: &str) -> Result<String, Error<!>> {
    let mut real_file = File::open(path)?;
    let temp_file_name = format!("{path}_{}", new_file_id());
    let mut temp_file = File::create(&temp_file_name)?;
    copy(&mut real_file, &mut temp_file)?;
    Ok(temp_file_name)
}

/// Enumeration of possible post-checks.
#[derive(Debug, PartialEq, Eq)]
pub enum PostCheck {
    /// Runs a post-check for the ext family.
    ///
    /// Precisely, this will run `e2fsck -fvn <file>`.
    Ext,

    /// Runs no post-check.
    None,
}

impl PostCheck {
    pub fn run(self, file_path: &str) -> Result<(), Error<!>> {
        match self {
            Self::Ext => {
                let output = Command::new("e2fsck").arg("-f").arg("-v").arg("-n").arg(file_path).output()?;

                if output.status.success() {
                    Ok(())
                } else {
                    Err(Error::IO(deku::no_std_io::Error::new(
                        deku::no_std_io::ErrorKind::Other,
                        String::from_utf8(output.stdout)
                            .unwrap_or_else(|_| unreachable!("e2fsck's output is valid UTF-8")),
                    )))
                }
            },
            Self::None => Ok(()),
        }
    }
}

/// Produces a new function that will be tested from a function whose arguments depend on the type of the test.
///
/// This macro should only be used inside an other module (usually `generated`).
///
/// Here are the arguments:
/// * `func_name`: name of the initial function ;
/// * `input_file`: name of the file containing the filesystem ;
/// * `post_check`: variant of the [`PostCheck`] enumeration to use ;
/// * `clear`: boolean indicating whether to clear the test file produced or not.
macro_rules! generate_fs_test {
    ($func_name:ident, $input_file:tt) => {
        generate_fs_test!($func_name, $input_file, crate::tests::PostCheck::None, true);
    };
    ($func_name:ident, $input_file:tt, $post_check:expr) => {
        generate_fs_test!($func_name, $input_file, $post_check, true);
    };
    ($func_name:ident, $input_file:tt, $post_check:expr, $clear:expr) => {
        #[test]
        fn $func_name() {
            let file_name = crate::tests::copy_file($input_file).unwrap();
            let file = std::fs::OpenOptions::new().read(true).write(true).open(&file_name).unwrap();
            super::$func_name(file);
            match $post_check.run(&file_name) {
                Ok(()) => {
                    if $clear {
                        std::fs::remove_file(&file_name).unwrap();
                    }
                },
                Err(err) => {
                    if $clear {
                        std::fs::remove_file(&file_name).unwrap();
                    }
                    panic!("Post Check Error: {err}");
                },
            }
        }
    };
}

pub(crate) use generate_fs_test;
