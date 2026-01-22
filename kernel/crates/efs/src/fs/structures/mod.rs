//! General structures found in several filesystems.

pub mod bitmap;
pub mod block;
#[cfg(feature = "ext2")]
pub mod indirection;
