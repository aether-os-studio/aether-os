/// A unique number used internally by the kernel to identify CPUs.
///
/// This is usually but not necessarily the same as the APIC ID.

// TODO: Differentiate between logical CPU IDs and hardware CPU IDs (e.g. APIC IDs)
#[derive(Clone, Copy, Eq, PartialEq, Hash)]
// TODO: NonMaxUsize?
// TODO: Optimize away this type if not cfg!(feature = "multi_core")
pub struct LogicalCpuId(u32);

impl LogicalCpuId {
    pub const BSP: Self = Self::new(0);

    pub const fn new(inner: u32) -> Self {
        Self(inner)
    }
    pub const fn get(self) -> u32 {
        self.0
    }
}

impl core::fmt::Debug for LogicalCpuId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "[logical cpu #{}]", self.0)
    }
}
impl core::fmt::Display for LogicalCpuId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "#{}", self.0)
    }
}
