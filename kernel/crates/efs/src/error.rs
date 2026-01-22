//! Interface for `efs` possible errors.

use alloc::string::ToString;

use derive_more::derive::{Display, From};

use crate::arch::ArchError;
use crate::fs::error::FsError;
use crate::path::PathError;

/// Enumeration of possible sources of error.
#[allow(clippy::error_impl_error)]
#[derive(Debug, Display, From)]
#[display("Error: {_variant}")]
pub enum Error<FSE: core::error::Error> {
    /// Architecture error.
    Arch(ArchError),

    /// Filesystem error.
    Fs(FsError<FSE>),

    /// Path error.
    Path(PathError),

    /// I/O error.
    IO(deku::no_std_io::Error),
}

impl<FSE: core::error::Error> core::error::Error for Error<FSE> {}

impl<FSE: core::error::Error> Error<FSE> {
    /// Converts an error that is not an [`FsError::Implementation`] into itself, for any filesystem error.
    #[must_use]
    pub fn from_infallible(err: Error<!>) -> Self {
        match err {
            Error::Arch(arch_error) => Self::Arch(arch_error),
            Error::Fs(fs_error) => Self::Fs(FsError::<FSE>::from_infallible(fs_error)),
            Error::Path(path_error) => Self::Path(path_error),
            Error::IO(io_error) => Self::IO(io_error),
        }
    }
}

impl<FSE: core::error::Error> From<Error<FSE>> for deku::no_std_io::Error {
    fn from(value: Error<FSE>) -> Self {
        match value {
            Error::Arch(arch_error) => Self::new(deku::no_std_io::ErrorKind::Other, arch_error.to_string()),
            Error::Fs(fs_error) => Self::new(deku::no_std_io::ErrorKind::Other, fs_error.to_string()),
            Error::Path(path_error) => Self::new(deku::no_std_io::ErrorKind::Other, path_error.to_string()),
            Error::IO(error) => error,
        }
    }
}
