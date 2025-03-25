// Copyright 2014 The Rust Project Developers. See the COPYRIGHT
// file at the top-level directory of this distribution and at
// http://rust-lang.org/COPYRIGHT.
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/licenses/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option. This file may not be copied, modified, or distributed
// except according to those terms.

/// The macro to invoke a syscall.
///
/// # Examples
///
/// The following code will print `Hello, world!` to the screen with black background and white foreground.
/// ```
/// syscall!(SYS_PUTSTRING, "Hello, world!\n", 0x00ffffff, 0x00000000);
/// ```
///
/// # Note
///
/// This macro is not meant to be used directly, but rather as a dependency for other crates.
#[macro_export]
macro_rules! syscall {
    ($nr:ident) => {
        ::asc::syscall0(::asc::nr::$nr)
    };

    ($nr:ident, $a1:expr) => {
        ::asc::syscall1(::asc::nr::$nr, $a1 as usize)
    };

    ($nr:ident, $a1:expr, $a2:expr) => {
        ::asc::syscall2(::asc::nr::$nr, $a1 as usize, $a2 as usize)
    };

    ($nr:ident, $a1:expr, $a2:expr, $a3:expr) => {
        ::asc::syscall3(::asc::nr::$nr, $a1 as usize, $a2 as usize, $a3 as usize)
    };

    ($nr:ident, $a1:expr, $a2:expr, $a3:expr, $a4:expr) => {
        ::asc::syscall4(
            ::asc::nr::$nr,
            $a1 as usize,
            $a2 as usize,
            $a3 as usize,
            $a4 as usize,
        )
    };

    ($nr:ident, $a1:expr, $a2:expr, $a3:expr, $a4:expr, $a5:expr) => {
        ::asc::syscall5(
            ::asc::nr::$nr,
            $a1 as usize,
            $a2 as usize,
            $a3 as usize,
            $a4 as usize,
            $a5 as usize,
        )
    };

    ($nr:ident, $a1:expr, $a2:expr, $a3:expr, $a4:expr, $a5:expr, $a6:expr) => {
        ::asc::syscall6(
            ::asc::nr::$nr,
            $a1 as usize,
            $a2 as usize,
            $a3 as usize,
            $a4 as usize,
            $a5 as usize,
            $a6 as usize,
        )
    };

    ($nr:ident, $a1:expr, $a2:expr, $a3:expr, $a4:expr, $a5:expr, $a6:expr, $a7:expr) => {
        ::asc::syscall7(
            ::asc::nr::$nr,
            $a1 as usize,
            $a2 as usize,
            $a3 as usize,
            $a4 as usize,
            $a5 as usize,
            $a6 as usize,
            $a7 as usize,
        )
    };
}
