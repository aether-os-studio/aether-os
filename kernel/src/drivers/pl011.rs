use core::{fmt, ptr::NonNull};

use acpi::sdt::spcr::{Spcr, SpcrInterfaceType};
use alloc::{string::ToString, sync::Arc};
use arm_pl011_uart::{LineConfig, PL011Registers, UniqueMmioPointer};
use rmm::{Arch, PageMapper, PhysicalAddress};
use spin::Mutex;

use crate::arch::CurrentRmmArch;
use crate::arch::rmm::page_flags;
use crate::drivers::{
    acpi::ACPI_TABLES,
    device::{CharDevice, MmioDevice},
    dtb::{DtbDevice, iter_device_by_compatible},
};
use crate::init::memory::{FRAME_ALLOCATOR, PAGE_SIZE, align_down, align_up};

pub struct Pl011Device<'a> {
    pl011: arm_pl011_uart::Uart<'a>,
}

impl<'a> Pl011Device<'a> {
    pub fn new(
        address: usize,
        data_bits: arm_pl011_uart::DataBits,
        parity: arm_pl011_uart::Parity,
        stop_bits: arm_pl011_uart::StopBits,
        baud_rate: u32,
        sysclk: u32,
    ) -> Self {
        let physical_address = align_down(address);
        let offset = address - physical_address;
        let physical_address = PhysicalAddress::new(physical_address);
        let virtual_address = unsafe { CurrentRmmArch::phys_to_virt(physical_address) };
        let size = align_up(PAGE_SIZE);

        let mut frame_allocator = FRAME_ALLOCATOR.lock();
        let mut mapper =
            unsafe { KernelPageMapper::current(rmm::TableKind::Kernel, &mut *frame_allocator) };

        for i in (0..size).step_by(PAGE_SIZE) {
            if let Some(flusher) = unsafe {
                mapper.map_phys(
                    virtual_address.add(i),
                    physical_address.add(i),
                    page_flags(virtual_address.add(i)),
                )
            } {
                flusher.flush();
            }
        }

        let mut pl011 = arm_pl011_uart::Uart::new(unsafe {
            UniqueMmioPointer::new(NonNull::new_unchecked(
                virtual_address.add(offset).data() as *mut PL011Registers
            ))
        });
        pl011
            .enable(
                LineConfig {
                    data_bits,
                    parity,
                    stop_bits,
                },
                baud_rate,
                sysclk,
            )
            .unwrap();
        Self { pl011 }
    }
}

impl CharDevice for Pl011Device<'_> {
    fn read(&mut self, buf: &mut [u8]) {
        for bp in buf.iter_mut() {
            loop {
                if let Some(b) = self.pl011.read_word().unwrap() {
                    *bp = b;
                    break;
                }
            }
        }
    }

    fn write(&mut self, buf: &[u8]) {
        for b in buf.iter() {
            self.pl011.write_word(*b);
        }
    }
}

impl fmt::Write for Pl011Device<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write(s.as_bytes());
        Ok(())
    }
}

pub static MAIN_PL011_DEVICE: Mutex<Option<Pl011Device>> = Mutex::new(None);

fn dtb_pl011_callback(device: Arc<DtbDevice>) {
    let mmio_ranges = device.get_mmio_addr_range().unwrap();
    let addr = mmio_ranges[0].0;
    let pl011 = Pl011Device::new(
        addr.data(),
        arm_pl011_uart::DataBits::Bits8,
        arm_pl011_uart::Parity::None,
        arm_pl011_uart::StopBits::One,
        115200,
        24000000,
    );
    let mut main_pl011_device = MAIN_PL011_DEVICE.lock();
    if main_pl011_device.is_none() {
        *main_pl011_device = Some(pl011);
    }
}

pub fn init() {
    if let Some(acpi_table) = ACPI_TABLES.lock().as_mut()
        && let Some(table) = acpi_table.find_table::<Spcr>()
    {
        if let SpcrInterfaceType::ArmPL011 = table.interface_type() {
            let pl011 = Pl011Device::new(
                table.base_address().unwrap().unwrap().address as usize,
                arm_pl011_uart::DataBits::Bits8,
                arm_pl011_uart::Parity::None,
                arm_pl011_uart::StopBits::One,
                115200,
                24000000,
            );
            let mut main_pl011_device = MAIN_PL011_DEVICE.lock();
            if main_pl011_device.is_none() {
                *main_pl011_device = Some(pl011);
            }
        }
    }
    iter_device_by_compatible("arm,pl011".to_string(), dtb_pl011_callback);
}

#[macro_export]
macro_rules! serial_print {
    ($($arg:tt)*) => ($crate::drivers::pl011::_print(format_args!($($arg)*)));
}

#[macro_export]
macro_rules! serial_println {
    () => ($crate::serial_print!("\n"));
    ($($arg:tt)*) => ($crate::serial_print!("{}\n", format_args!($($arg)*)));
}

#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    use core::fmt::Write;
    let mut main_device = MAIN_PL011_DEVICE.lock();
    if let Some(main_device) = main_device.as_mut() {
        main_device.write_fmt(args).unwrap();
    }
}
