//! Implementation of the [name strings](../index.html#name-strings) of SFS.
//!
//! It is closely linked to the POSIX paths, whose implementation is available [in this crate](crate::path::Path).
//!
//! See the [official documentation](https://web.archive.org/web/20170315134201/https://www.d-rift.nl/combuster/vdisk/sfs.html#Name_Strings) for more information.

use alloc::borrow::ToOwned;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt::Display;
use core::str::FromStr;

use spin::Lazy;

use super::error::SfsError;
use crate::error::Error;
use crate::fs::error::FsError;
use crate::path::{Path, PathError, UnixStr};

/// [`NameString`] of the root directory.
pub static ROOT_NAME_STRING: Lazy<NameString> = Lazy::new(|| NameString(String::from("/")));

/// Returns whether the given character is forbidden in SFS name strings.
///
/// The full list of forbidden character is the following:
///
/// - Characters whose code is strictly below `0x20`
///
/// - Characters whose code is included between `0x80` and `0x9F` (inclusive).
///
/// - The following special characters: `"` (double quote, `0x22`), `*` (asterix, `0x2A`), `:` (colon, `0x3A`), `<`
///   (less than sign, `0x3C`), `>` (greater than sign, `0x3E`), `?` (question mark, `0x3F`), `\` (backward slash,
///   `0x5C`), `<DEL>` (delete, `0x7F`) and `<NBSP>` (no-break space, `0xA0`).
///
/// In particular, the `/` character **is allowed** and is expected to be present in volume names as a directory
/// separator only. Thus, it is not allowed within a volume label.
#[must_use]
pub const fn is_forbidden_character(c: &u8) -> bool {
    matches!(c, 0x00..0x20 | 0x80..=0x9F | b'"' | b'*' | b':' | b'<' | b'>' | b'?' | b'\\' | 0x7F | 0xA0)
}

/// Checks whether the given byte sequence is valid, meaning that no forbidden character appears, and that the last
/// character is `<NUL>` (`\0`).
#[must_use]
pub fn is_valid_name_string(str: &[u8]) -> bool {
    if str.len() <= 1 {
        return false;
    }

    for c in str.iter().rev().skip(1).rev() {
        if is_forbidden_character(c) {
            return false;
        }
    }

    str.last().is_some_and(|c| *c == b'\0')
}

/// A SFS name string.
///
/// A [`NameString`] cannot contain any forbidden character (see the list [here](is_forbidden_character)) except for the
/// last character that is expected to be a `<NUL>` character (`\0`). It is guaranteed at creation time.
///
/// It is very similar to an absolute [`Path`], but does not contain the initial '/' character. That's why the
/// conversion functions `from` and `into` [`UnixStr`] and [`Path`] takes this into account.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NameString(String);

impl NameString {
    /// Returns a new valid [`NameString`] from the given sequence of bytes.
    ///
    /// # Errors
    ///
    /// Returns a [`SfsError::InvalidNameString`] error if the given sequence of bytes is not a valid name string.
    pub fn new(str: Vec<u8>) -> Result<Self, Error<SfsError>> {
        if is_valid_name_string(&str)
            && let Ok(inner) = String::from_utf8(str.clone())
        {
            // SAFETY: Any valid sequence ends with a <NUL> character
            Ok(Self(unsafe { inner.strip_suffix('\0').unwrap_unchecked().to_owned() }))
        } else {
            Err(Error::Fs(FsError::Implementation(SfsError::InvalidNameString(str))))
        }
    }

    /// Returns a new valid [`NameString`] from the start of the given sequence of bytes, ending at the first `<NUL>`
    /// character encountered.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`new`](NameString::new).
    pub fn new_from_start(seq: &[u8]) -> Result<Self, Error<SfsError>> {
        seq.iter().position(|byte| *byte == 0).map_or_else(
            || {
                let mut seq_with_nul = seq.to_vec();
                seq_with_nul.push(0);
                Self::new(seq_with_nul)
            },
            |idx| Self::new(seq[..=idx].to_vec()),
        )
    }

    /// Appends to the given [`NameString`] the other one.
    pub fn join(&mut self, other: &Self) {
        self.0.push_str(&other.0);
    }
}

impl FromStr for NameString {
    type Err = Error<SfsError>;

    /// Parses a string and returns a [`NameString`] if it is a valid name string.
    ///
    /// This string must not contain any `<NUL>` character (even at the end): it is directly handled without for
    /// practicity.
    ///
    /// # Errors
    ///
    /// Returns an [`SfsError::InvalidNameString`] if the given string is an invalid name string.
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut bytes = s.as_bytes().to_vec();
        bytes.push(b'\0');
        if is_valid_name_string(&bytes) {
            Ok(Self(s.to_owned()))
        } else {
            Err(Error::Fs(FsError::Implementation(SfsError::InvalidNameString(bytes))))
        }
    }
}

impl From<NameString> for UnixStr<'_> {
    fn from(value: NameString) -> Self {
        let full_path = format!("/{}", value.0);
        // SAFETY: Any valid NamedString is a valid UnixStr
        unsafe { UnixStr::from_str(&full_path).unwrap_unchecked() }
    }
}

impl From<NameString> for Path<'_> {
    fn from(value: NameString) -> Self {
        UnixStr::from(value).into()
    }
}

impl TryFrom<Path<'_>> for NameString {
    type Error = Error<SfsError>;

    fn try_from(path: Path<'_>) -> Result<Self, Self::Error> {
        if !path.is_absolute() {
            return Err(Error::Path(PathError::AbsolutePathRequired(path.to_string())));
        }

        let path_str = path.to_string();
        // SAFETY: the path is absolute so it starts with a `/`
        let stripped_path = unsafe { path_str.strip_prefix("/").unwrap_unchecked() };

        let bytes = stripped_path.as_bytes();
        if bytes.is_empty() || bytes.iter().any(is_forbidden_character) {
            Err(Error::Fs(FsError::Implementation(SfsError::InvalidNameString(bytes.to_vec()))))
        } else {
            Ok(Self(stripped_path.to_owned()))
        }
    }
}

impl Display for NameString {
    fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(fmt, "{}", UnixStr::from(self.clone()))
    }
}

#[cfg(test)]
mod test {
    use core::str::FromStr;

    use super::NameString;
    use crate::path::Path;

    #[test]
    fn name_string_creation() {
        assert!(NameString::new(b"".to_vec()).is_err());
        assert!(NameString::new(b"\0".to_vec()).is_err());

        assert!(NameString::new(b"foo\0".to_vec()).is_ok());
        assert!(NameString::new(b"foo".to_vec()).is_err());
        assert!(NameString::new(b"foo*.txt\0".to_vec()).is_err());

        assert!(NameString::from_str("foo").is_ok());
        assert!(NameString::from_str("foo.txt").is_ok());
        assert!(NameString::from_str("foo/bar").is_ok());
        assert!(NameString::from_str("foo/bar/baz.txt").is_ok());

        assert!(NameString::from_str("foo.txt\0").is_err());
        assert!(NameString::from_str("foo:txt").is_err());
        assert!(NameString::from_str("foo*txt").is_err());
    }

    #[test]
    fn name_string_to_path() {
        assert_eq!(Path::from(NameString::from_str("foo.txt").unwrap()), Path::from_str("/foo.txt").unwrap());
        assert_eq!(Path::from(NameString::from_str("foo/bar.txt").unwrap()), Path::from_str("/foo/bar.txt").unwrap());
    }

    #[test]
    fn path_to_name_string() {
        assert_eq!(
            NameString::try_from(Path::from_str("/foo.txt").unwrap()).unwrap(),
            NameString::from_str("foo.txt").unwrap()
        );
        assert_eq!(
            NameString::try_from(Path::from_str("/foo/bar.txt").unwrap()).unwrap(),
            NameString::from_str("foo/bar.txt").unwrap()
        );
    }

    #[test]
    fn name_string_from_start() {
        assert_eq!(
            NameString::new_from_start(b"foo.txt\0*:zeaqqdqs#").unwrap(),
            NameString::from_str("foo.txt").unwrap()
        );
        assert_eq!(NameString::new_from_start(b"foo.txttoto").unwrap(), NameString::from_str("foo.txttoto").unwrap());
        assert!(NameString::new_from_start(b"foo.txt*\0:zeaqqdqs#").is_err());
    }
}
