use crate::drivers::bus::usb::USB_DEVICES;

pub mod xhci;

pub fn init() {
    xhci::init();
    USB_DEVICES
        .lock()
        .iter()
        .for_each(|usbdev| debug!("{}", usbdev.read()));
}
