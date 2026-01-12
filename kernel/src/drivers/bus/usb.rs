use core::ops::{Deref, DerefMut};

use alloc::sync::{Arc, Weak};
use spin::RwLock;

#[inline]
pub const fn fls(x: u32) -> u32 {
    if x == 0 { 0 } else { 32 - x.leading_zeros() }
}

pub fn usb_get_period(device: ArcUsbDevice, epdesc: UsbEndpointDescriptor) -> u32 {
    let period = epdesc.interval as u32;
    if device.read().speed != UsbDeviceSpeed::High {
        if period <= 0 { 0 } else { fls(period) }
    } else {
        if period <= 4 { 0 } else { period - 4 }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum UsbDir {
    Out = 0x00,
    In = 0x80,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct UsbEndpointDescriptor {
    pub length: u8,
    pub desc_type: u8,

    pub ep_addr: u8,
    pub attr: u8,
    pub max_packet_size: u16,
    pub interval: u8,
}

pub trait UsbHcd {
    fn realloc_pipe(
        &mut self,
        device: ArcUsbDevice,
        pipe: Option<ArcUsbPipe>,
        epdesc: Option<UsbEndpointDescriptor>,
    ) -> Option<ArcUsbPipe>;

    fn read_pipe(&mut self, pipe: ArcUsbPipe, cmd: Option<&[u8]>, data: Option<&mut [u8]>);
    fn write_pipe(&mut self, pipe: ArcUsbPipe, cmd: Option<&[u8]>, data: Option<&[u8]>);

    fn read_intr_pipe(&mut self, pipe: ArcUsbPipe, data: &mut [u8], cb: fn(u64));
    fn write_intr_pipe(&mut self, pipe: ArcUsbPipe, data: &[u8], cb: fn(u64));
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsbDeviceSpeed {
    Unknown,
    Full = 1,
    Low,
    High,
    Super,
}

pub type ArcUsbPipe = Arc<RwLock<UsbPipe>>;
pub type WeakUsbPipe = Weak<RwLock<UsbPipe>>;

#[derive(Clone, Default)]
pub struct UsbPipe {
    pub slot: u8,
    pub epid: u8,
    pub maxpacket: u16,
    pub eptype: u8,
}

pub type ArcUsbDevice = Arc<RwLock<UsbDevice>>;
pub type WeakUsbDevice = Weak<RwLock<UsbDevice>>;

#[derive(Clone)]
pub struct UsbDevice {
    pub hub: ArcUsbHub,
    pub ctrl_pipe: ArcUsbPipe,
    pub slot: u8,
    pub port: u8,
    pub product_id: u16,
    pub vendor_id: u16,
    pub speed: UsbDeviceSpeed,
    pub devaddr: u8,
}

pub type ArcUsbHub = Arc<RwLock<UsbHub>>;
pub type WeakUsbHub = Weak<RwLock<UsbHub>>;

#[derive(Clone)]
pub struct UsbHub {
    pub usbdev: Option<ArcUsbDevice>,
    pub port: u8,
    pub portcount: u8,
}

#[repr(C)]
#[derive(Clone, Default)]
pub struct UsbCtrlRequest {
    pub req_type: u8,
    pub req: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}

impl Deref for UsbCtrlRequest {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        unsafe { core::slice::from_raw_parts(self as *const _ as *const u8, size_of::<Self>()) }
    }
}

impl DerefMut for UsbCtrlRequest {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { core::slice::from_raw_parts_mut(self as *mut _ as *mut u8, size_of::<Self>()) }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct UsbDeviceDescriptor {
    pub length: u8,
    pub desc_type: u8,

    pub bcd_usb: u16,
    pub dev_class: u8,
    pub dev_subclass: u8,
    pub dev_protocol: u8,
    pub max_packet_size_0: u8,
    pub id_vendor: u16,
    pub id_product: u16,
    pub bcd_device: u16,
    pub i_manufacturer: u8,
    pub i_product: u8,
    pub i_serial_number: u8,
    pub b_num_configurations: u8,
}

impl Deref for UsbDeviceDescriptor {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        unsafe { core::slice::from_raw_parts(self as *const _ as *const u8, size_of::<Self>()) }
    }
}

impl DerefMut for UsbDeviceDescriptor {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { core::slice::from_raw_parts_mut(self as *mut _ as *mut u8, size_of::<Self>()) }
    }
}
