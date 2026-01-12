use core::{ptr::NonNull, sync::atomic::AtomicBool};

use acpi::sdt::madt::{Madt, MadtEntry};
use arm_gic::{
    IntId, UniqueMmioPointer,
    gicv2::{GicV2, registers::Gicc},
    gicv3::{GicV3, registers::GicrSgi},
};
use rmm::{Arch, PageFlags, PageMapper, PhysicalAddress};
use spin::Mutex;

use crate::{
    arch::{CurrentRmmArch, irq::IrqControllerArch, smp::get_mpidr},
    drivers::acpi::ACPI_TABLES,
    init::memory::{FRAME_ALLOCATOR, PAGE_SIZE, align_down, align_up},
    smp::{BSP_CPUARCHID, CPU_COUNT, get_cpuid_by_archid},
};

pub enum Gic<'a> {
    V2(GicV2<'a>),
    V3(GicV3<'a>),
}

impl<'a> IrqControllerArch for Gic<'a> {
    fn enable_irq(&mut self, irq: usize) {
        let irq = irq as u32;
        let intid = if irq > 0 && irq < 16 {
            IntId::sgi(irq)
        } else if irq > 16 && irq < 32 {
            IntId::ppi(irq - 16)
        } else if irq > 32 && irq < 1020 {
            IntId::spi(irq - 32)
        } else {
            panic!("Invalid irq {}", irq);
        };
        let mpidr = get_mpidr();
        let cpuid = get_cpuid_by_archid(mpidr as usize);
        match self {
            Gic::V2(v2) => {
                let _ = v2.enable_interrupt(intid, true);
            }
            Gic::V3(v3) => {
                let _ = v3.enable_interrupt(intid, Some(cpuid), true);
            }
        }
    }

    fn disable_irq(&mut self, irq: usize) {
        let irq = irq as u32;
        let intid = if irq > 0 && irq < 16 {
            IntId::sgi(irq)
        } else if irq > 16 && irq < 32 {
            IntId::ppi(irq - 16)
        } else if irq > 32 && irq < 1020 {
            IntId::spi(irq - 32)
        } else {
            panic!("Invalid irq {}", irq);
        };
        let mpidr = get_mpidr();
        let cpuid = get_cpuid_by_archid(mpidr as usize);
        match self {
            Gic::V2(v2) => {
                let _ = v2.enable_interrupt(intid, false);
            }
            Gic::V3(v3) => {
                let _ = v3.enable_interrupt(intid, Some(cpuid), false);
            }
        }
    }

    fn send_eoi(&mut self, _irq: usize) {}
}

pub static GIC_IRQ_CONTROLLER: Mutex<Option<Gic<'static>>> = Mutex::new(None);
pub static GIC_IRQ_INITIALIZED: AtomicBool = AtomicBool::new(false);

pub fn init() {
    let mut have_gicr = false;
    let mut gic_version = 0;
    let mut gicc_base = 0;
    let mut gicr_base = 0;
    let mut gicd_base = 0;

    if let Some(acpi_table) = ACPI_TABLES.lock().as_mut() {
        // Initialize by ACPI
        let madt = acpi_table.find_table::<Madt>();
        if let Some(madt) = madt {
            for entry in madt.get().entries() {
                match entry {
                    MadtEntry::Gicd(gicd) => {
                        gic_version = gicd.gic_version;

                        let mut frame_allocator = FRAME_ALLOCATOR.lock();
                        let mut mapper = unsafe {
                            PageMapper::current(rmm::TableKind::Kernel, &mut *frame_allocator)
                        };

                        let start = align_down(gicd.physical_base_address as usize);
                        let end = align_up(start + PAGE_SIZE * 16);

                        for addr in (start..end).step_by(PAGE_SIZE) {
                            let phys = PhysicalAddress::new(addr);
                            let virt = unsafe { CurrentRmmArch::phys_to_virt(phys) };
                            let flags = PageFlags::<CurrentRmmArch>::new()
                                .write(true)
                                .custom_flag(1 << 2, true);
                            if let Some(flusher) = unsafe { mapper.map_phys(virt, phys, flags) } {
                                flusher.flush();
                            }
                        }

                        let phys = PhysicalAddress::new(gicd.physical_base_address as usize);
                        let virt = unsafe { CurrentRmmArch::phys_to_virt(phys) };
                        gicd_base = virt.data() as usize;
                    }
                    MadtEntry::GicRedistributor(gicr) => {
                        if gicr_base == 0 {
                            have_gicr = true;
                            let mut frame_allocator = FRAME_ALLOCATOR.lock();
                            let mut mapper = unsafe {
                                PageMapper::current(rmm::TableKind::Kernel, &mut *frame_allocator)
                            };

                            let start = align_down(gicr.discovery_range_base_address as usize);
                            let end = align_up(
                                start
                                    + PAGE_SIZE
                                        * 32
                                        * CPU_COUNT.load(core::sync::atomic::Ordering::SeqCst),
                            );

                            for addr in (start..end).step_by(PAGE_SIZE) {
                                let phys = PhysicalAddress::new(addr);
                                let virt = unsafe { CurrentRmmArch::phys_to_virt(phys) };
                                let flags = PageFlags::<CurrentRmmArch>::new()
                                    .write(true)
                                    .custom_flag(1 << 2, true);
                                if let Some(flusher) = unsafe { mapper.map_phys(virt, phys, flags) }
                                {
                                    flusher.flush();
                                }
                            }

                            let phys =
                                PhysicalAddress::new(gicr.discovery_range_base_address as usize);
                            let virt = unsafe { CurrentRmmArch::phys_to_virt(phys) };

                            gicr_base = virt.data() as usize;
                        }
                    }
                    MadtEntry::Gicc(gicc) => {
                        if gicc_base == 0 {
                            let mut frame_allocator = FRAME_ALLOCATOR.lock();
                            let mut mapper = unsafe {
                                PageMapper::current(rmm::TableKind::Kernel, &mut *frame_allocator)
                            };

                            let start = align_down(gicc.gic_registers_address as usize);
                            let end = align_up(start + PAGE_SIZE * 2);

                            for addr in (start..end).step_by(PAGE_SIZE) {
                                let phys = PhysicalAddress::new(addr);
                                let virt = unsafe { CurrentRmmArch::phys_to_virt(phys) };
                                let flags = PageFlags::<CurrentRmmArch>::new()
                                    .write(true)
                                    .custom_flag(1 << 2, true);
                                if let Some(flusher) = unsafe { mapper.map_phys(virt, phys, flags) }
                                {
                                    flusher.flush();
                                }
                            }

                            let phys = PhysicalAddress::new(gicc.gic_registers_address as usize);
                            let virt = unsafe { CurrentRmmArch::phys_to_virt(phys) };

                            gicc_base = virt.data() as usize;
                        }
                    }
                    _ => {}
                }
            }

            if have_gicr && gic_version < 3 {
                gic_version = 3;
            } else {
                gic_version = 2;
            }

            if gic_version == 2 {
                let mut gicv2 = unsafe {
                    GicV2::new(
                        gicd_base as *mut arm_gic::gicv2::registers::Gicd,
                        gicc_base as *mut Gicc,
                    )
                };
                gicv2.setup();
                gicv2.enable_all_interrupts(true);

                let gic = Gic::V2(gicv2);
                *GIC_IRQ_CONTROLLER.lock() = Some(gic);
            } else {
                let gicd = unsafe {
                    UniqueMmioPointer::new(
                        NonNull::new(gicd_base as *mut arm_gic::gicv3::registers::Gicd).unwrap(),
                    )
                };
                let gicr = NonNull::new(gicr_base as *mut GicrSgi).unwrap();
                let mut gicv3 = unsafe {
                    GicV3::new(
                        gicd,
                        gicr,
                        CPU_COUNT.load(core::sync::atomic::Ordering::SeqCst),
                        gic_version == 4,
                    )
                };
                gicv3.setup(BSP_CPUARCHID.load(core::sync::atomic::Ordering::SeqCst));
                gicv3.enable_all_interrupts(true);

                let gic = Gic::V3(gicv3);
                *GIC_IRQ_CONTROLLER.lock() = Some(gic);
            }

            GIC_IRQ_INITIALIZED.store(true, core::sync::atomic::Ordering::SeqCst);
        }
    }
}
