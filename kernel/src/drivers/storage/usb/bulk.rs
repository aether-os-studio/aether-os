use alloc::{string::String, sync::Arc};
use spin::RwLock;

use crate::{
    arch::{CurrentTimeArch, time::TimeArch},
    drivers::{
        bus::usb::{ArcUsbDevice, ArcUsbHcd, ArcUsbPipe, USB_DEVICES, UsbEndpointDescriptor},
        storage::{BLOCK_DEVICES, BlockDevice, BlockDeviceTrait},
    },
};

// USB Mass Storage Class codes
pub const USB_CLASS_MASS_STORAGE: u8 = 0x08;
pub const USB_SUBCLASS_SCSI: u8 = 0x06;
pub const USB_PROTOCOL_BULK_ONLY: u8 = 0x50;

// CBW/CSW Signatures
pub const CBW_SIGNATURE: u32 = 0x43425355; // "USBC"
pub const CSW_SIGNATURE: u32 = 0x53425355; // "USBS"

// CBW Direction flags
pub const CBW_FLAG_DATA_IN: u8 = 0x80;
pub const CBW_FLAG_DATA_OUT: u8 = 0x00;

// CSW Status
pub const CSW_STATUS_PASSED: u8 = 0x00;
pub const CSW_STATUS_FAILED: u8 = 0x01;
pub const CSW_STATUS_PHASE_ERROR: u8 = 0x02;

// SCSI Commands
pub const SCSI_TEST_UNIT_READY: u8 = 0x00;
pub const SCSI_REQUEST_SENSE: u8 = 0x03;
pub const SCSI_INQUIRY: u8 = 0x12;
pub const SCSI_READ_CAPACITY_10: u8 = 0x25;
pub const SCSI_READ_CAPACITY_16: u8 = 0x9E; // Service Action = 0x10
pub const SCSI_READ_10: u8 = 0x28;
pub const SCSI_READ_12: u8 = 0xA8;
pub const SCSI_READ_16: u8 = 0x88;
pub const SCSI_WRITE_10: u8 = 0x2A;
pub const SCSI_WRITE_12: u8 = 0xAA;
pub const SCSI_WRITE_16: u8 = 0x8A;

// Service Actions for READ CAPACITY (16)
pub const SAI_READ_CAPACITY_16: u8 = 0x10;

// Endpoint types
pub const USB_ENDPOINT_XFER_BULK: u8 = 0x02;

// 容量阈值
pub const MAX_LBA_32: u64 = 0xFFFFFFFF;
pub const MAX_BLOCKS_10: u32 = 0xFFFF;
pub const MAX_BLOCKS_12: u32 = 0xFFFFFFFF;

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct CommandBlockWrapper {
    pub signature: u32,
    pub tag: u32,
    pub data_transfer_length: u32,
    pub flags: u8,
    pub lun: u8,
    pub cb_length: u8,
    pub cb: [u8; 16],
}

impl CommandBlockWrapper {
    pub fn new(tag: u32, data_len: u32, flags: u8, lun: u8, cb: &[u8]) -> Self {
        let mut cbw = Self {
            signature: CBW_SIGNATURE.to_le(),
            tag: tag.to_le(),
            data_transfer_length: data_len.to_le(),
            flags,
            lun,
            cb_length: cb.len().min(16) as u8,
            cb: [0u8; 16],
        };
        let len = cb.len().min(16);
        cbw.cb[..len].copy_from_slice(&cb[..len]);
        cbw
    }
}

impl CommandBlockWrapper {
    pub const SIZE: usize = 31;

    pub fn as_bytes(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self as *const _ as *const u8, Self::SIZE) }
    }
}

impl Default for CommandBlockWrapper {
    fn default() -> Self {
        Self {
            signature: CBW_SIGNATURE,
            tag: 0,
            data_transfer_length: 0,
            flags: 0,
            lun: 0,
            cb_length: 0,
            cb: [0u8; 16],
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct CommandStatusWrapper {
    pub signature: u32,
    pub tag: u32,
    pub data_residue: u32,
    pub status: u8,
}

impl CommandStatusWrapper {
    pub const SIZE: usize = 13;

    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self as *mut _ as *mut u8, Self::SIZE) }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct ScsiInquiryResponse {
    pub peripheral: u8,
    pub removable: u8,
    pub version: u8,
    pub response_format: u8,
    pub additional_length: u8,
    pub flags: [u8; 3],
    pub vendor: [u8; 8],
    pub product: [u8; 16],
    pub revision: [u8; 4],
}

impl ScsiInquiryResponse {
    pub const SIZE: usize = 36;

    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self as *mut _ as *mut u8, Self::SIZE) }
    }

    pub fn vendor_string(&self) -> &str {
        core::str::from_utf8(&self.vendor).unwrap_or("").trim()
    }

    pub fn product_string(&self) -> &str {
        core::str::from_utf8(&self.product).unwrap_or("").trim()
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct ScsiReadCapacity10Response {
    pub last_lba: [u8; 4],   // Big-endian
    pub block_size: [u8; 4], // Big-endian
}

impl ScsiReadCapacity10Response {
    pub const SIZE: usize = 8;

    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self as *mut _ as *mut u8, Self::SIZE) }
    }

    pub fn get_last_lba(&self) -> u32 {
        u32::from_be_bytes(self.last_lba)
    }

    pub fn get_block_size(&self) -> u32 {
        u32::from_be_bytes(self.block_size)
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct ScsiReadCapacity16Response {
    pub last_lba: [u8; 8],   // Big-endian, 64-bit
    pub block_size: [u8; 4], // Big-endian
    pub flags: u8,
    pub lowest_aligned_lba: [u8; 2],
    pub reserved: [u8; 16],
}

impl ScsiReadCapacity16Response {
    pub const SIZE: usize = 32;

    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self as *mut _ as *mut u8, Self::SIZE) }
    }

    pub fn get_last_lba(&self) -> u64 {
        u64::from_be_bytes(self.last_lba)
    }

    pub fn get_block_size(&self) -> u32 {
        u32::from_be_bytes(self.block_size)
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct ScsiRequestSenseResponse {
    pub response_code: u8,
    pub segment_number: u8,
    pub sense_key: u8,
    pub information: [u8; 4],
    pub additional_sense_length: u8,
    pub command_specific: [u8; 4],
    pub asc: u8,
    pub ascq: u8,
    pub fru_code: u8,
    pub sense_key_specific: [u8; 3],
}

impl ScsiRequestSenseResponse {
    pub const SIZE: usize = 18;

    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self as *mut _ as *mut u8, Self::SIZE) }
    }

    pub fn sense_key(&self) -> u8 {
        self.sense_key & 0x0F
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScsiCommandSize {
    Cmd10,
    Cmd12,
    Cmd16,
}

impl ScsiCommandSize {
    pub fn select(lba: u64, block_count: u32) -> Self {
        if lba > MAX_LBA_32 {
            ScsiCommandSize::Cmd16
        } else if block_count > MAX_BLOCKS_10 {
            ScsiCommandSize::Cmd12
        } else {
            ScsiCommandSize::Cmd10
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum MassStorageError {
    NoInterface,
    NoEndpoint,
    PipeAllocationFailed,
    CommandFailed {
        status: u8,
        sense_key: u8,
        asc: u8,
        ascq: u8,
    },
    ProtocolError,
    DeviceNotReady,
    InvalidParameter,
    MediaError,
    WriteProtected,
}

fn msc_err_to_str(e: MassStorageError) -> &'static str {
    match e {
        MassStorageError::InvalidParameter => "Invalid Argument",
        MassStorageError::DeviceNotReady => "Device Not Ready",
        _ => "Generic Error",
    }
}

pub type Result<T> = core::result::Result<T, MassStorageError>;

pub type ArcUsbMassStorage = Arc<RwLock<UsbMassStorage>>;

pub struct UsbMassStorage {
    pub device: ArcUsbDevice,
    pub hcd: ArcUsbHcd,
    pub bulk_in: ArcUsbPipe,
    pub bulk_out: ArcUsbPipe,
    pub lun: u8,
    pub max_lun: u8,
    tag: u32,

    // 设备信息
    pub block_size: u32,
    pub block_count: u64,
    pub vendor: String,
    pub product: String,

    pub use_16byte_commands: bool,
}

impl UsbMassStorage {
    pub fn new(device: ArcUsbDevice) -> Result<Self> {
        let iface = {
            let dev = device.read();
            dev.ifaces
                .iter()
                .find(|iface| {
                    let iface = iface.read();
                    iface.desc.interface_class == USB_CLASS_MASS_STORAGE
                        && iface.desc.interface_subclass == USB_SUBCLASS_SCSI
                        && iface.desc.interface_protocol == USB_PROTOCOL_BULK_ONLY
                })
                .cloned()
        }
        .ok_or(MassStorageError::NoInterface)?;

        let mut bulk_in_ep: Option<UsbEndpointDescriptor> = None;
        let mut bulk_out_ep: Option<UsbEndpointDescriptor> = None;

        {
            let iface = iface.read();
            for ep in iface.endpoints.iter() {
                if (ep.attr & 0x03) == USB_ENDPOINT_XFER_BULK {
                    if (ep.ep_addr & 0x80) != 0 {
                        bulk_in_ep = Some(*ep);
                    } else {
                        bulk_out_ep = Some(*ep);
                    }
                }
            }
        }

        let bulk_in_ep = bulk_in_ep.ok_or(MassStorageError::NoEndpoint)?;
        let bulk_out_ep = bulk_out_ep.ok_or(MassStorageError::NoEndpoint)?;

        let hcd = device.read().hub.read().hcd.clone();

        let bulk_in = hcd
            .write()
            .realloc_pipe(device.clone(), None, Some(bulk_in_ep))
            .ok_or(MassStorageError::PipeAllocationFailed)?;

        let bulk_out = hcd
            .write()
            .realloc_pipe(device.clone(), None, Some(bulk_out_ep))
            .ok_or(MassStorageError::PipeAllocationFailed)?;

        let mut storage = Self {
            device,
            hcd,
            bulk_in,
            bulk_out,
            lun: 0,
            max_lun: 0,
            tag: 1,
            block_size: 512,
            block_count: 0,
            vendor: String::new(),
            product: String::new(),
            use_16byte_commands: false,
        };

        storage.init()?;

        Ok(storage)
    }

    fn next_tag(&mut self) -> u32 {
        let tag = self.tag;
        self.tag = self.tag.wrapping_add(1);
        if self.tag == 0 {
            self.tag = 1;
        }
        tag
    }

    fn send_cbw(&mut self, cbw: &CommandBlockWrapper) {
        self.hcd
            .write()
            .write_pipe(self.bulk_out.clone(), None, Some(cbw.as_bytes()));
        self.hcd.write().wait_pipe(self.bulk_out.clone()).unwrap();
    }

    fn receive_csw(&mut self) -> CommandStatusWrapper {
        let mut csw = CommandStatusWrapper::default();
        self.hcd
            .write()
            .read_pipe(self.bulk_in.clone(), None, Some(csw.as_bytes_mut()));
        self.hcd.write().wait_pipe(self.bulk_in.clone()).unwrap();
        csw
    }

    fn transfer_data_in(&mut self, data: &mut [u8]) {
        self.hcd
            .write()
            .read_pipe(self.bulk_in.clone(), None, Some(data));
        self.hcd.write().wait_pipe(self.bulk_in.clone()).unwrap();
    }

    fn transfer_data_out(&mut self, data: &[u8]) {
        self.hcd
            .write()
            .write_pipe(self.bulk_out.clone(), None, Some(data));
        self.hcd.write().wait_pipe(self.bulk_out.clone()).unwrap();
    }

    fn execute_command(
        &mut self,
        cb: &[u8],
        data_in: Option<&mut [u8]>,
        data_out: Option<&[u8]>,
    ) -> Result<()> {
        let result = self.execute_command_raw(cb, data_in, data_out);

        if let Err(MassStorageError::CommandFailed { .. }) = result
            && let Ok(sense) = self.request_sense()
        {
            return Err(MassStorageError::CommandFailed {
                status: CSW_STATUS_FAILED,
                sense_key: sense.sense_key(),
                asc: sense.asc,
                ascq: sense.ascq,
            });
        }

        result
    }

    fn execute_command_raw(
        &mut self,
        cb: &[u8],
        data_in: Option<&mut [u8]>,
        data_out: Option<&[u8]>,
    ) -> Result<()> {
        let tag = self.next_tag();

        let (data_len, flags) = if let Some(ref data) = data_in {
            (data.len() as u32, CBW_FLAG_DATA_IN)
        } else if let Some(data) = data_out {
            (data.len() as u32, CBW_FLAG_DATA_OUT)
        } else {
            (0, 0)
        };

        let cbw = CommandBlockWrapper::new(tag, data_len, flags, self.lun, cb);
        self.send_cbw(&cbw);

        if let Some(data) = data_in {
            self.transfer_data_in(data);
        } else if let Some(data) = data_out {
            self.transfer_data_out(data);
        }

        let csw = self.receive_csw();

        if csw.signature != CSW_SIGNATURE || csw.tag != tag {
            return Err(MassStorageError::ProtocolError);
        }

        match csw.status {
            CSW_STATUS_PASSED => Ok(()),
            CSW_STATUS_FAILED => Err(MassStorageError::CommandFailed {
                status: csw.status,
                sense_key: 0,
                asc: 0,
                ascq: 0,
            }),
            CSW_STATUS_PHASE_ERROR => Err(MassStorageError::ProtocolError),
            _ => Err(MassStorageError::CommandFailed {
                status: csw.status,
                sense_key: 0,
                asc: 0,
                ascq: 0,
            }),
        }
    }

    pub fn test_unit_ready(&mut self) -> Result<()> {
        let cb = [SCSI_TEST_UNIT_READY, 0, 0, 0, 0, 0];
        self.execute_command(&cb, None, None)
    }

    pub fn inquiry(&mut self) -> Result<ScsiInquiryResponse> {
        let mut response = ScsiInquiryResponse::default();
        let cb = [SCSI_INQUIRY, 0, 0, 0, ScsiInquiryResponse::SIZE as u8, 0];
        self.execute_command(&cb, Some(response.as_bytes_mut()), None)?;
        Ok(response)
    }

    pub fn request_sense(&mut self) -> Result<ScsiRequestSenseResponse> {
        let mut response = ScsiRequestSenseResponse::default();
        let cb = [
            SCSI_REQUEST_SENSE,
            0,
            0,
            0,
            ScsiRequestSenseResponse::SIZE as u8,
            0,
        ];
        self.execute_command_raw(&cb, Some(response.as_bytes_mut()), None)?;
        Ok(response)
    }

    pub fn read_capacity_10(&mut self) -> Result<(u64, u32)> {
        let mut response = ScsiReadCapacity10Response::default();
        let cb = [SCSI_READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        self.execute_command(&cb, Some(response.as_bytes_mut()), None)?;

        let last_lba = response.get_last_lba();
        let block_size = response.get_block_size();

        Ok((last_lba as u64 + 1, block_size))
    }

    pub fn read_capacity_16(&mut self) -> Result<(u64, u32)> {
        let mut response = ScsiReadCapacity16Response::default();
        let alloc_len = ScsiReadCapacity16Response::SIZE as u32;

        let cb = [
            SCSI_READ_CAPACITY_16,
            SAI_READ_CAPACITY_16, // Service Action
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0, // LBA (保留)
            (alloc_len >> 24) as u8,
            (alloc_len >> 16) as u8,
            (alloc_len >> 8) as u8,
            alloc_len as u8,
            0, // Reserved
            0, // Control
        ];

        self.execute_command(&cb, Some(response.as_bytes_mut()), None)?;

        let last_lba = response.get_last_lba();
        let block_size = response.get_block_size();

        Ok((last_lba + 1, block_size))
    }

    pub fn read_capacity(&mut self) -> Result<(u64, u32)> {
        let (block_count, block_size) = self.read_capacity_10()?;

        if block_count > MAX_LBA_32 {
            self.use_16byte_commands = true;
            return self.read_capacity_16();
        }

        Ok((block_count, block_size))
    }

    fn read_10(
        &mut self,
        lba: u32,
        count: u16,
        buffer: &mut [u8],
    ) -> core::result::Result<(), &'static str> {
        let cb = [
            SCSI_READ_10,
            0,
            (lba >> 24) as u8,
            (lba >> 16) as u8,
            (lba >> 8) as u8,
            lba as u8,
            0, // Group number
            (count >> 8) as u8,
            count as u8,
            0, // Control
        ];
        self.execute_command(&cb, Some(buffer), None)
            .map_err(msc_err_to_str)
    }

    fn read_12(
        &mut self,
        lba: u32,
        count: u32,
        buffer: &mut [u8],
    ) -> core::result::Result<(), &'static str> {
        let cb = [
            SCSI_READ_12,
            0,
            (lba >> 24) as u8,
            (lba >> 16) as u8,
            (lba >> 8) as u8,
            lba as u8,
            (count >> 24) as u8,
            (count >> 16) as u8,
            (count >> 8) as u8,
            count as u8,
            0, // Group number
            0, // Control
        ];
        self.execute_command(&cb, Some(buffer), None)
            .map_err(msc_err_to_str)
    }

    fn read_16(
        &mut self,
        lba: u64,
        count: u32,
        buffer: &mut [u8],
    ) -> core::result::Result<(), &'static str> {
        let cb = [
            SCSI_READ_16,
            0,
            (lba >> 56) as u8,
            (lba >> 48) as u8,
            (lba >> 40) as u8,
            (lba >> 32) as u8,
            (lba >> 24) as u8,
            (lba >> 16) as u8,
            (lba >> 8) as u8,
            lba as u8,
            (count >> 24) as u8,
            (count >> 16) as u8,
            (count >> 8) as u8,
            count as u8,
            0, // Group number
            0, // Control
        ];
        self.execute_command(&cb, Some(buffer), None)
            .map_err(msc_err_to_str)
    }

    pub fn read_blocks(
        &mut self,
        lba: u64,
        count: u32,
        buffer: &mut [u8],
    ) -> core::result::Result<(), &'static str> {
        let required_len = count as usize * self.block_size as usize;
        if buffer.len() < required_len {
            return Err("Invalid Argument");
        }

        let buffer = &mut buffer[..required_len];

        match ScsiCommandSize::select(lba, count) {
            ScsiCommandSize::Cmd10 => self.read_10(lba as u32, count as u16, buffer),
            ScsiCommandSize::Cmd12 => self.read_12(lba as u32, count, buffer),
            ScsiCommandSize::Cmd16 => self.read_16(lba, count, buffer),
        }
    }

    fn write_10(
        &mut self,
        lba: u32,
        count: u16,
        buffer: &[u8],
    ) -> core::result::Result<(), &'static str> {
        let cb = [
            SCSI_WRITE_10,
            0,
            (lba >> 24) as u8,
            (lba >> 16) as u8,
            (lba >> 8) as u8,
            lba as u8,
            0, // Group number
            (count >> 8) as u8,
            count as u8,
            0, // Control
        ];
        self.execute_command(&cb, None, Some(buffer))
            .map_err(msc_err_to_str)
    }

    fn write_12(
        &mut self,
        lba: u32,
        count: u32,
        buffer: &[u8],
    ) -> core::result::Result<(), &'static str> {
        let cb = [
            SCSI_WRITE_12,
            0,
            (lba >> 24) as u8,
            (lba >> 16) as u8,
            (lba >> 8) as u8,
            lba as u8,
            (count >> 24) as u8,
            (count >> 16) as u8,
            (count >> 8) as u8,
            count as u8,
            0, // Group number
            0, // Control
        ];
        self.execute_command(&cb, None, Some(buffer))
            .map_err(msc_err_to_str)
    }

    fn write_16(
        &mut self,
        lba: u64,
        count: u32,
        buffer: &[u8],
    ) -> core::result::Result<(), &'static str> {
        let cb = [
            SCSI_WRITE_16,
            0,
            (lba >> 56) as u8,
            (lba >> 48) as u8,
            (lba >> 40) as u8,
            (lba >> 32) as u8,
            (lba >> 24) as u8,
            (lba >> 16) as u8,
            (lba >> 8) as u8,
            lba as u8,
            (count >> 24) as u8,
            (count >> 16) as u8,
            (count >> 8) as u8,
            count as u8,
            0, // Group number
            0, // Control
        ];
        self.execute_command(&cb, None, Some(buffer))
            .map_err(msc_err_to_str)
    }

    pub fn write_blocks(
        &mut self,
        lba: u64,
        count: u32,
        buffer: &[u8],
    ) -> core::result::Result<(), &'static str> {
        let required_len = count as usize * self.block_size as usize;
        if buffer.len() < required_len {
            return Err("Invalid Argument");
        }

        let buffer = &buffer[..required_len];

        match ScsiCommandSize::select(lba, count) {
            ScsiCommandSize::Cmd10 => self.write_10(lba as u32, count as u16, buffer),
            ScsiCommandSize::Cmd12 => self.write_12(lba as u32, count, buffer),
            ScsiCommandSize::Cmd16 => self.write_16(lba, count, buffer),
        }
    }

    pub fn read_blocks_chunked(
        &mut self,
        start_lba: u64,
        total_count: u64,
        buffer: &mut [u8],
    ) -> core::result::Result<(), &'static str> {
        let required_len = total_count as usize * self.block_size as usize;
        if buffer.len() < required_len {
            return Err("Invalid Argument");
        }

        let max_blocks_per_transfer = 128u32;

        let mut current_lba = start_lba;
        let mut remaining = total_count;
        let mut offset = 0usize;

        while remaining > 0 {
            let blocks_to_read = (remaining as u32).min(max_blocks_per_transfer);
            let bytes_to_read = blocks_to_read as usize * self.block_size as usize;

            self.read_blocks(
                current_lba,
                blocks_to_read,
                &mut buffer[offset..offset + bytes_to_read],
            )?;

            current_lba += blocks_to_read as u64;
            remaining -= blocks_to_read as u64;
            offset += bytes_to_read;
        }

        Ok(())
    }

    pub fn write_blocks_chunked(
        &mut self,
        start_lba: u64,
        total_count: u64,
        buffer: &[u8],
    ) -> core::result::Result<(), &'static str> {
        let required_len = total_count as usize * self.block_size as usize;
        if buffer.len() < required_len {
            return Err("Invalid Argument");
        }

        let max_blocks_per_transfer = 128u32;

        let mut current_lba = start_lba;
        let mut remaining = total_count;
        let mut offset = 0usize;

        while remaining > 0 {
            let blocks_to_write = (remaining as u32).min(max_blocks_per_transfer);
            let bytes_to_write = blocks_to_write as usize * self.block_size as usize;

            self.write_blocks(
                current_lba,
                blocks_to_write,
                &buffer[offset..offset + bytes_to_write],
            )?;

            current_lba += blocks_to_write as u64;
            remaining -= blocks_to_write as u64;
            offset += bytes_to_write;
        }

        Ok(())
    }

    fn init(&mut self) -> Result<()> {
        let mut ready = false;
        for _ in 0..10 {
            CurrentTimeArch::delay(10000000);
            match self.test_unit_ready() {
                Ok(()) => {
                    ready = true;
                    break;
                }
                Err(_) => {
                    let _ = self.request_sense();
                }
            }
        }

        if !ready {
            return Err(MassStorageError::DeviceNotReady);
        }

        let inquiry = self.inquiry()?;
        let vendor = core::str::from_utf8(&inquiry.vendor).unwrap_or("").trim();
        let product = core::str::from_utf8(&inquiry.product).unwrap_or("").trim();
        self.vendor.push_str(vendor);
        self.product.push_str(product);

        let (block_count, block_size) = self.read_capacity()?;
        self.block_count = block_count;
        self.block_size = block_size;

        Ok(())
    }

    pub fn capacity(&self) -> u64 {
        self.block_count * self.block_size as u64
    }

    pub fn read(
        &mut self,
        offset: u64,
        buffer: &mut [u8],
    ) -> core::result::Result<usize, &'static str> {
        if buffer.is_empty() {
            return Ok(0);
        }

        let start_lba = offset / self.block_size as u64;
        let offset_in_block = (offset % self.block_size as u64) as usize;
        let end_offset = offset + buffer.len() as u64;
        let end_lba = (end_offset - 1) / self.block_size as u64;
        let block_count = end_lba - start_lba + 1;

        let mut temp = vec![0u8; block_count as usize * self.block_size as usize];

        self.read_blocks_chunked(start_lba, block_count, &mut temp)?;

        buffer.copy_from_slice(&temp[offset_in_block..offset_in_block + buffer.len()]);

        Ok(buffer.len())
    }

    pub fn write(
        &mut self,
        offset: u64,
        buffer: &[u8],
    ) -> core::result::Result<usize, &'static str> {
        if buffer.is_empty() {
            return Ok(0);
        }

        let start_lba = offset / self.block_size as u64;
        let offset_in_block = (offset % self.block_size as u64) as usize;
        let end_offset = offset + buffer.len() as u64;
        let end_lba = (end_offset - 1) / self.block_size as u64;
        let block_count = end_lba - start_lba + 1;

        let mut temp = vec![0u8; block_count as usize * self.block_size as usize];

        if offset_in_block != 0 {
            self.read_blocks(start_lba, 1, &mut temp[..self.block_size as usize])?;
        }

        let end_offset_in_block = (end_offset % self.block_size as u64) as usize;
        if end_offset_in_block != 0 && end_lba != start_lba {
            let last_block_offset = (block_count - 1) as usize * self.block_size as usize;
            self.read_blocks(
                end_lba,
                1,
                &mut temp[last_block_offset..last_block_offset + self.block_size as usize],
            )?;
        }

        temp[offset_in_block..offset_in_block + buffer.len()].copy_from_slice(buffer);

        self.write_blocks_chunked(start_lba, block_count, &temp)?;

        Ok(buffer.len())
    }
}

impl BlockDeviceTrait for UsbMassStorage {
    fn read_block(
        &mut self,
        block_id: u64,
        buf: &mut [u8],
    ) -> core::result::Result<(), &'static str> {
        self.read_blocks_chunked(
            block_id,
            (buf.len() as u32).div_ceil(self.block_size) as u64,
            buf,
        )
    }

    fn write_block(&mut self, block_id: u64, buf: &[u8]) -> core::result::Result<(), &'static str> {
        self.write_blocks_chunked(
            block_id,
            (buf.len() as u32).div_ceil(self.block_size) as u64,
            buf,
        )
    }

    fn block_size(&self) -> u32 {
        self.block_size
    }

    fn block_count(&self) -> u64 {
        self.block_count
    }
}

pub fn init() {
    for usbdev in USB_DEVICES.lock().iter() {
        for iface in usbdev.read().ifaces.iter() {
            if iface.read().desc.interface_class == USB_CLASS_MASS_STORAGE
                && iface.read().desc.interface_subclass == USB_SUBCLASS_SCSI
                && iface.read().desc.interface_protocol == USB_PROTOCOL_BULK_ONLY
                && let Some(msc_dev) = match UsbMassStorage::new(usbdev.clone()) {
                    Ok(storage) => Some(Arc::new(RwLock::new(storage))),
                    Err(e) => {
                        warn!("USB Mass Storage probe failed: {:?}", e);
                        None
                    }
                }
            {
                BLOCK_DEVICES.lock().push(BlockDevice::new(msc_dev));
            }
        }
    }
}
