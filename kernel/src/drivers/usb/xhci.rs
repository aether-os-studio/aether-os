use core::{hint::spin_loop, num::NonZeroUsize};

use alloc::{collections::btree_map::BTreeMap, sync::Arc, vec::Vec};
use bit_field::BitField;
use rmm::{
    Arch, FrameAllocator, FrameCount, PageFlags, PageMapper, PhysicalAddress, VirtualAddress,
};
use spin::{Mutex, RwLock};
use xhci::accessor::Mapper;

use crate::{
    arch::{CurrentRmmArch, CurrentTimeArch, time::TimeArch},
    drivers::bus::{
        pci::PCI_DEVICES,
        usb::{
            ArcUsbDevice, ArcUsbHub, ArcUsbPipe, USB_DEVICES, UsbConfigDescriptor, UsbCtrlRequest,
            UsbDevice, UsbDeviceDescriptor, UsbDeviceSpeed, UsbEndpointDescriptor, UsbHcd, UsbHub,
            UsbInterfaceDescriptor, UsbPipe, usb_get_period,
        },
    },
    init::memory::{FRAME_ALLOCATOR, PAGE_SIZE, align_down, align_up},
};

#[repr(u8)]
pub enum StreamContextType {
    SecondaryRing,
    PrimaryRing,
    PrimarySsa8,
    PrimarySsa16,
    PrimarySsa32,
    PrimarySsa64,
    PrimarySsa128,
    PrimarySsa256,
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum TrbType {
    Reserved,
    /* Transfer */
    Normal,
    SetupStage,
    DataStage,
    StatusStage,
    Isoch,
    Link,
    EventData,
    NoOp,
    /* Command */
    EnableSlot,
    DisableSlot,
    AddressDevice,
    ConfigureEndpoint,
    EvaluateContext,
    ResetEndpoint,
    StopEndpoint,
    SetTrDequeuePointer,
    ResetDevice,
    ForceEvent,
    NegotiateBandwidth,
    SetLatencyToleranceValue,
    GetPortBandwidth,
    ForceHeader,
    NoOpCmd,
    /* Reserved */
    GetExtendedProperty,
    SetExtendedProperty,
    Rsv26,
    Rsv27,
    Rsv28,
    Rsv29,
    Rsv30,
    Rsv31,
    /* Events */
    Transfer,
    CommandCompletion,
    PortStatusChange,
    BandwidthRequest,
    Doorbell,
    HostController,
    DeviceNotification,
    MfindexWrap,
    /* Reserved from 40 to 47, vendor devined from 48 to 63 */
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum TrbCompletionCode {
    Invalid = 0x00,
    Success = 0x01,
    DataBuffer = 0x02,
    BabbleDetected = 0x03,
    UsbTransaction = 0x04,
    Trb = 0x05,
    Stall = 0x06,
    Resource = 0x07,
    Bandwidth = 0x08,
    NoSlotsAvailable = 0x09,
    InvalidStreamType = 0x0A,
    SlotNotEnabled = 0x0B,
    EndpointNotEnabled = 0x0C,
    ShortPacket = 0x0D,
    RingUnderrun = 0x0E,
    RingOverrun = 0x0F,
    VfEventRingFull = 0x10,
    Parameter = 0x11,
    BandwidthOverrun = 0x12,
    ContextState = 0x13,
    NoPingResponse = 0x14,
    EventRingFull = 0x15,
    IncompatibleDevice = 0x16,
    MissedService = 0x17,
    CommandRingStopped = 0x18,
    CommandAborted = 0x19,
    Stopped = 0x1A,
    StoppedLengthInvalid = 0x1B,
    StoppedShortPacket = 0x1C,
    MaxExitLatencyTooLarge = 0x1D,
    Rsv30 = 0x1E,
    IsochBuffer = 0x1F,
    EventLost = 0x20,
    Undefined = 0x21,
    InvalidStreamId = 0x22,
    SecondaryBandwidth = 0x23,
    SplitTransaction = 0x24,
    /* Values from 37 to 191 are reserved */
    /* 192 to 223 are vendor defined errors */
    /* 224 to 255 are vendor defined information */
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TransferKind {
    NoData,
    Reserved,
    Out,
    In,
}

#[repr(C)]
#[derive(Clone, Default)]
pub struct XhciTrb {
    pub data_low: u32,
    pub data_high: u32,
    pub status: u32,
    pub control: u32,
}

pub const TRB_STATUS_COMPLETION_CODE_SHIFT: u8 = 24;
pub const TRB_STATUS_COMPLETION_CODE_MASK: u32 = 0xFF00_0000;

pub const TRB_STATUS_COMPLETION_PARAM_SHIFT: u8 = 0;
pub const TRB_STATUS_COMPLETION_PARAM_MASK: u32 = 0x00FF_FFFF;

pub const TRB_STATUS_TRANSFER_LENGTH_SHIFT: u8 = 0;
pub const TRB_STATUS_TRANSFER_LENGTH_MASK: u32 = 0x00FF_FFFF;

pub const TRB_CONTROL_TRB_TYPE_SHIFT: u8 = 10;
pub const TRB_CONTROL_TRB_TYPE_MASK: u32 = 0x0000_FC00;

pub const TRB_CONTROL_EVENT_DATA_SHIFT: u8 = 2;
pub const TRB_CONTROL_EVENT_DATA_BIT: u32 = 1 << TRB_CONTROL_EVENT_DATA_SHIFT;

pub const TRB_CONTROL_ENDPOINT_ID_MASK: u32 = 0x001F_0000;
pub const TRB_CONTROL_ENDPOINT_ID_SHIFT: u8 = 16;

impl XhciTrb {
    pub fn set(&mut self, data: u64, status: u32, control: u32) {
        self.data_low = data as u32;
        self.data_high = (data >> 32) as u32;
        self.status = status;
        self.control = control;
    }

    pub fn reserved(&mut self, cycle: bool) {
        self.set(0, 0, ((TrbType::Reserved as u32) << 10) | (cycle as u32));
    }

    pub fn read_data(&self) -> u64 {
        (self.data_low as u64) | ((self.data_high as u64) << 32)
    }

    pub fn completion_code(&self) -> u8 {
        (self.status >> TRB_STATUS_COMPLETION_CODE_SHIFT) as u8
    }
    pub fn completion_param(&self) -> u32 {
        self.status & TRB_STATUS_COMPLETION_PARAM_MASK
    }
    fn has_completion_trb_pointer(&self) -> bool {
        if self.completion_code() == TrbCompletionCode::RingUnderrun as u8
            || self.completion_code() == TrbCompletionCode::RingOverrun as u8
        {
            false
        } else if self.completion_code() == TrbCompletionCode::VfEventRingFull as u8 {
            false
        } else {
            true
        }
    }
    pub fn completion_trb_pointer(&self) -> Option<u64> {
        debug_assert_eq!(self.trb_type(), TrbType::CommandCompletion as u8);

        if self.has_completion_trb_pointer() {
            Some(self.read_data())
        } else {
            None
        }
    }
    pub fn transfer_event_trb_pointer(&self) -> Option<u64> {
        debug_assert_eq!(self.trb_type(), TrbType::Transfer as u8);

        if self.has_completion_trb_pointer() {
            Some(self.read_data())
        } else {
            None
        }
    }

    pub fn port_status_change_port_id(&self) -> Option<u8> {
        debug_assert_eq!(self.trb_type(), TrbType::PortStatusChange as u8);

        if self.has_completion_trb_pointer() {
            let data = self.read_data();
            Some(((data >> 24) & 0xFF) as u8)
        } else {
            None
        }
    }

    pub fn event_slot(&self) -> u8 {
        (self.control >> 24) as u8
    }
    pub fn transfer_length(&self) -> u32 {
        self.status & TRB_STATUS_TRANSFER_LENGTH_MASK
    }
    pub fn event_data_bit(&self) -> bool {
        self.control.get_bit(TRB_CONTROL_EVENT_DATA_SHIFT as usize)
    }
    pub fn event_data(&self) -> Option<u64> {
        if self.event_data_bit() {
            Some(self.read_data())
        } else {
            None
        }
    }
    pub fn endpoint_id(&self) -> u8 {
        ((self.control & TRB_CONTROL_ENDPOINT_ID_MASK) >> TRB_CONTROL_ENDPOINT_ID_SHIFT) as u8
    }
    pub fn trb_type(&self) -> u8 {
        ((self.control & TRB_CONTROL_TRB_TYPE_MASK) >> TRB_CONTROL_TRB_TYPE_SHIFT) as u8
    }

    pub fn link(&mut self, address: usize, toggle: bool, cycle: bool) {
        self.set(
            address as u64,
            0,
            ((TrbType::Link as u32) << 10) | ((toggle as u32) << 1) | (cycle as u32),
        );
    }

    pub fn no_op_cmd(&mut self, cycle: bool) {
        self.set(0, 0, ((TrbType::NoOpCmd as u32) << 10) | (cycle as u32));
    }

    pub fn enable_slot(&mut self, slot_type: u8, cycle: bool) {
        self.set(
            0,
            0,
            (((slot_type as u32) & 0x1F) << 16)
                | ((TrbType::EnableSlot as u32) << 10)
                | (cycle as u32),
        );
    }
    pub fn disable_slot(&mut self, slot: u8, cycle: bool) {
        self.set(
            0,
            0,
            (u32::from(slot) << 24) | ((TrbType::DisableSlot as u32) << 10) | u32::from(cycle),
        );
    }

    pub fn address_device(&mut self, slot_id: u8, input_ctx_ptr: usize, bsr: bool, cycle: bool) {
        assert_eq!(
            (input_ctx_ptr as u64) & 0xFFFF_FFFF_FFFF_FFF0,
            input_ctx_ptr as u64,
            "unaligned input context ptr"
        );
        self.set(
            input_ctx_ptr as u64,
            0,
            (u32::from(slot_id) << 24)
                | ((TrbType::AddressDevice as u32) << 10)
                | (u32::from(bsr) << 9)
                | u32::from(cycle),
        );
    }
    pub fn configure_endpoint(&mut self, slot_id: u8, input_ctx_ptr: usize, cycle: bool) {
        assert_eq!(
            (input_ctx_ptr as u64) & 0xFFFF_FFFF_FFFF_FFF0,
            input_ctx_ptr as u64,
            "unaligned input context ptr"
        );

        self.set(
            input_ctx_ptr as u64,
            0,
            (u32::from(slot_id) << 24)
                | ((TrbType::ConfigureEndpoint as u32) << 10)
                | u32::from(cycle),
        );
    }
    pub fn evaluate_context(&mut self, slot_id: u8, input_ctx_ptr: usize, bsr: bool, cycle: bool) {
        assert_eq!(
            (input_ctx_ptr as u64) & 0xFFFF_FFFF_FFFF_FFF0,
            input_ctx_ptr as u64,
            "unaligned input context ptr"
        );
        self.set(
            input_ctx_ptr as u64,
            0,
            (u32::from(slot_id) << 24)
                | ((TrbType::EvaluateContext as u32) << 10)
                | (u32::from(bsr) << 9)
                | u32::from(cycle),
        );
    }
    pub fn reset_endpoint(&mut self, slot_id: u8, endp_num_xhc: u8, tsp: bool, cycle: bool) {
        assert_eq!(endp_num_xhc & 0x1F, endp_num_xhc);
        self.set(
            0,
            0,
            (u32::from(slot_id) << 24)
                | (u32::from(endp_num_xhc) << 16)
                | ((TrbType::ResetEndpoint as u32) << 10)
                | (u32::from(tsp) << 9)
                | u32::from(cycle),
        );
    }
    pub fn set_tr_deque_ptr(
        &mut self,
        deque_ptr: u64,
        cycle: bool,
        sct: StreamContextType,
        stream_id: u16,
        endp_num_xhc: u8,
        slot: u8,
    ) {
        assert_eq!(deque_ptr & 0xFFFF_FFFF_FFFF_FFF1, deque_ptr);
        assert_eq!(endp_num_xhc & 0x1F, endp_num_xhc);

        self.set(
            deque_ptr | ((sct as u64) << 1),
            u32::from(stream_id) << 16,
            (u32::from(slot) << 24)
                | (u32::from(endp_num_xhc) << 16)
                | ((TrbType::SetTrDequeuePointer as u32) << 10)
                | u32::from(cycle),
        )
    }
    pub fn stop_endpoint(&mut self, slot_id: u8, endp_num_xhc: u8, suspend: bool, cycle: bool) {
        assert_eq!(endp_num_xhc & 0x1F, endp_num_xhc);
        self.set(
            0,
            0,
            (u32::from(slot_id) << 24)
                | (u32::from(suspend) << 23)
                | (u32::from(endp_num_xhc) << 16)
                | ((TrbType::StopEndpoint as u32) << 10)
                | u32::from(cycle),
        );
    }
    pub fn reset_device(&mut self, slot_id: u8, cycle: bool) {
        self.set(
            0,
            0,
            (u32::from(slot_id) << 24) | ((TrbType::ResetDevice as u32) << 10) | u32::from(cycle),
        );
    }

    pub fn transfer_no_op(&mut self, interrupter: u8, ent: bool, ch: bool, ioc: bool, cycle: bool) {
        self.set(
            0,
            u32::from(interrupter) << 22,
            ((TrbType::NoOp as u32) << 10)
                | (u32::from(ioc) << 5)
                | (u32::from(ch) << 4)
                | (u32::from(ent) << 1)
                | u32::from(cycle),
        );
    }

    pub fn setup(&mut self, setup: u64, transfer: TransferKind, cycle: bool) {
        self.set(
            setup,
            8,
            ((transfer as u32) << 16)
                | ((TrbType::SetupStage as u32) << 10)
                | (1 << 6)
                | (cycle as u32),
        );
    }

    pub fn data(&mut self, buffer: usize, length: u16, input: bool, cycle: bool) {
        self.set(
            buffer as u64,
            length as u32,
            ((input as u32) << 16) | ((TrbType::DataStage as u32) << 10) | (cycle as u32),
        );
    }

    pub fn cycle(&self) -> bool {
        self.control.get_bit(0)
    }

    pub fn status(
        &mut self,
        interrupter: u16,
        input: bool,
        ioc: bool,
        ch: bool,
        ent: bool,
        cycle: bool,
    ) {
        self.set(
            0,
            u32::from(interrupter) << 22,
            (u32::from(input) << 16)
                | ((TrbType::StatusStage as u32) << 10)
                | (u32::from(ioc) << 5)
                | (u32::from(ch) << 4)
                | (u32::from(ent) << 1)
                | (cycle as u32),
        );
    }
    pub fn normal(
        &mut self,
        buffer: u64,
        len: u32,
        cycle: bool,
        estimated_td_size: u8,
        interrupter: u8,
        ent: bool,
        isp: bool,
        chain: bool,
        ioc: bool,
        idt: bool,
        bei: bool,
    ) {
        assert_eq!(estimated_td_size & 0x1F, estimated_td_size);
        self.set(
            buffer,
            len | (u32::from(estimated_td_size) << 17) | (u32::from(interrupter) << 22),
            u32::from(cycle)
                | (u32::from(ent) << 1)
                | (u32::from(isp) << 2)
                | (u32::from(chain) << 4)
                | (u32::from(ioc) << 5)
                | (u32::from(idt) << 6)
                | (u32::from(bei) << 9)
                | ((TrbType::Normal as u32) << 10),
        )
    }
    pub fn is_command_trb(&self) -> bool {
        let valid_trb_types = [
            TrbType::NoOpCmd as u8,
            TrbType::EnableSlot as u8,
            TrbType::DisableSlot as u8,
            TrbType::AddressDevice as u8,
            TrbType::ConfigureEndpoint as u8,
            TrbType::EvaluateContext as u8,
            TrbType::ResetEndpoint as u8,
            TrbType::StopEndpoint as u8,
            TrbType::SetTrDequeuePointer as u8,
            TrbType::ResetDevice as u8,
            TrbType::ForceEvent as u8,
            TrbType::NegotiateBandwidth as u8,
            TrbType::SetLatencyToleranceValue as u8,
            TrbType::GetPortBandwidth as u8,
            TrbType::ForceHeader as u8,
            TrbType::GetExtendedProperty as u8,
            TrbType::SetExtendedProperty as u8,
        ];
        valid_trb_types.contains(&self.trb_type())
    }
    pub fn is_transfer_trb(&self) -> bool {
        let valid_trb_types = [
            TrbType::Normal as u8,
            TrbType::SetupStage as u8,
            TrbType::DataStage as u8,
            TrbType::StatusStage as u8,
            TrbType::Isoch as u8,
            TrbType::NoOp as u8,
        ];
        valid_trb_types.contains(&self.trb_type())
    }
}

impl core::fmt::Debug for XhciTrb {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(
            f,
            "Trb {{ data: {:>016X}, status: {:>08X}, control: {:>08X} }}",
            self.read_data(),
            self.status,
            self.control
        )
    }
}

impl core::fmt::Display for XhciTrb {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(
            f,
            "({:>016X}, {:>08X}, {:>08X})",
            self.read_data(),
            self.status,
            self.control
        )
    }
}

pub struct XhciRing {
    ring_paddr: PhysicalAddress,
    idx: usize,
    event_idx: usize,
    cs: bool,
    result: XhciTrb,
}

impl XhciRing {
    pub fn new() -> Self {
        let ring_paddr = unsafe { FRAME_ALLOCATOR.lock().allocate_one() }
            .expect("No memory can be allocate for xhci ring");
        let ring_vaddr = unsafe { CurrentRmmArch::phys_to_virt(ring_paddr) };
        unsafe {
            core::ptr::write_bytes(
                ring_vaddr.data() as *mut u64,
                0,
                PAGE_SIZE / size_of::<u64>(),
            )
        };
        Self {
            ring_paddr,
            idx: 0,
            event_idx: 0,
            cs: true,
            result: XhciTrb::default(),
        }
    }

    pub fn trbs(&mut self) -> &'static mut [XhciTrb] {
        let virt = unsafe { CurrentRmmArch::phys_to_virt(self.ring_paddr) };
        unsafe {
            core::slice::from_raw_parts_mut(
                virt.data() as *mut XhciTrb,
                PAGE_SIZE / size_of::<XhciTrb>(),
            )
        }
    }

    pub fn submit_trb<F>(&mut self, cb: F)
    where
        F: FnOnce(&mut XhciTrb, bool),
    {
        let trbs = self.trbs();
        if self.idx >= (PAGE_SIZE / size_of::<XhciTrb>() - 1) {
            trbs[self.idx].link(self.ring_paddr.data() as usize, true, self.cs);
            self.idx = 0;
        }
        cb(&mut trbs[self.idx], self.cs);
        self.idx += 1;
    }
}

#[derive(Clone)]
struct XhciMapper;

impl Mapper for XhciMapper {
    unsafe fn map(&mut self, phys_start: usize, bytes: usize) -> NonZeroUsize {
        let phys = align_down(phys_start);
        let offset = phys_start - phys;
        let physical = PhysicalAddress::new(phys);
        let virtual_address = unsafe { CurrentRmmArch::phys_to_virt(physical) };
        let size = align_up(bytes);

        let mut frame_allocator = FRAME_ALLOCATOR.lock();
        let mut mapper = unsafe {
            PageMapper::<CurrentRmmArch, _>::current(rmm::TableKind::Kernel, &mut *frame_allocator)
        };

        for i in (0..size).step_by(PAGE_SIZE) {
            if let Some(flusher) = mapper.map_phys(
                virtual_address.add(i * PAGE_SIZE),
                physical.add(i * PAGE_SIZE),
                PageFlags::<CurrentRmmArch>::new().write(true),
            ) {
                flusher.flush();
            }
        }

        NonZeroUsize::new_unchecked(virtual_address.add(offset).data())
    }

    fn unmap(&mut self, _virt_start: usize, _bytes: usize) {}
}

pub type ArcXhciPipe = Arc<RwLock<XhciPipe>>;

pub struct XhciPipe {
    slot: u8,
    epid: u8,
    maxpacket: u16,
    ring: Mutex<XhciRing>,
}

#[repr(C)]
#[derive(Clone)]
pub struct XhciErSeg {
    ptr_low: u32,
    ptr_high: u32,
    size: u32,
    reserved: u32,
}

pub struct XhciHcd {
    registers: xhci::Registers<XhciMapper>,
    dcbaa_phys: PhysicalAddress,
    eseg_phys: PhysicalAddress,
    cmd_ring: Mutex<XhciRing>,
    event_ring: Mutex<XhciRing>,
    max_slots: u8,
    max_ports: u8,
    context64_shift: usize,
    pipes: BTreeMap<u8, BTreeMap<u8, ArcXhciPipe>>,
}

pub enum XhciSpeed {
    Unknown,
    Full = 1,
    Low = 2,
    High = 3,
    Super = 4,
}

fn usbspeed_to_xhcispeed(usbspeed: UsbDeviceSpeed) -> XhciSpeed {
    match usbspeed {
        UsbDeviceSpeed::Unknown => XhciSpeed::Unknown,
        UsbDeviceSpeed::Full => XhciSpeed::Full,
        UsbDeviceSpeed::Low => XhciSpeed::Low,
        UsbDeviceSpeed::High => XhciSpeed::High,
        UsbDeviceSpeed::Super => XhciSpeed::Super,
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct XhciInctx {
    del: u32,
    add: u32,
    reserved: [u32; 6],
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct XhciSlotctx {
    ctx: [u32; 4],
    reserved: [u32; 4],
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct XhciEpctx {
    ctx: [u32; 2],
    deq_low: u32,
    deq_high: u32,
    length: u32,
    reserved: [u32; 3],
}

impl XhciHcd {
    pub fn new(base: PhysicalAddress) -> Self {
        let dcbaa_phys = unsafe { FRAME_ALLOCATOR.lock().allocate_one() }
            .expect("No memory can be allocated for xhci device context base address array");
        let eseg_phys = unsafe { FRAME_ALLOCATOR.lock().allocate_one() }
            .expect("No memory can be allocated for xhci event segment");
        let mut xhci = Self {
            registers: unsafe { xhci::Registers::new(base.data(), XhciMapper) },
            dcbaa_phys,
            eseg_phys,
            cmd_ring: Mutex::new(XhciRing::new()),
            event_ring: Mutex::new(XhciRing::new()),
            max_slots: 0,
            max_ports: 0,
            context64_shift: 0,
            pipes: BTreeMap::new(),
        };
        xhci.init();
        xhci
    }

    pub fn init(&mut self) {
        let dcbaa_virt = unsafe { CurrentRmmArch::phys_to_virt(self.dcbaa_phys) };
        unsafe {
            core::ptr::write_bytes(
                dcbaa_virt.data() as *mut u64,
                0,
                PAGE_SIZE / size_of::<u64>(),
            )
        };
        let eseg_virt = unsafe { CurrentRmmArch::phys_to_virt(self.eseg_phys) };
        unsafe {
            core::ptr::write_bytes(
                eseg_virt.data() as *mut u64,
                0,
                PAGE_SIZE / size_of::<u64>(),
            )
        };

        let capability = &mut self.registers.capability;

        self.max_slots = capability
            .hcsparams1
            .read_volatile()
            .number_of_device_slots();
        self.max_ports = capability.hcsparams1.read_volatile().number_of_ports();
        self.context64_shift = if capability.hccparams1.read_volatile().context_size() {
            1
        } else {
            0
        };

        let operational = &mut self.registers.operational;
        operational.usbcmd.update_volatile(|cmd| {
            cmd.clear_run_stop();
        });
        while !operational.usbsts.read_volatile().hc_halted() {
            spin_loop();
        }

        operational.usbcmd.update_volatile(|cmd| {
            cmd.set_host_controller_reset();
        });
        while operational.usbcmd.read_volatile().host_controller_reset() {
            spin_loop();
        }
        while operational.usbsts.read_volatile().controller_not_ready() {
            spin_loop();
        }

        operational.config.update_volatile(|config| {
            config.set_max_device_slots_enabled(self.max_slots);
        });

        operational.dcbaap.update_volatile(|dcbaa| {
            dcbaa.set(self.dcbaa_phys.data() as u64);
        });
        operational.crcr.update_volatile(|crcr| {
            crcr.set_command_ring_pointer(self.cmd_ring.lock().ring_paddr.data() as u64);
            crcr.set_ring_cycle_state();
        });

        let interrupter = &mut self.registers.interrupter_register_set;
        // TODO: multi interrupters
        let mut interrupter0 = interrupter.interrupter_mut(0);
        interrupter0.erstsz.update_volatile(|erstsz| {
            erstsz.set(1);
        });
        let event_ring_paddr = self.event_ring.lock().ring_paddr.data() as u64;
        interrupter0.erdp.update_volatile(|erdp| {
            erdp.set_event_ring_dequeue_pointer(event_ring_paddr);
        });
        let er_seg = unsafe { (eseg_virt.data() as *mut XhciErSeg).as_mut_unchecked() };
        er_seg.ptr_low = event_ring_paddr as u32;
        er_seg.ptr_high = (event_ring_paddr >> 32) as u32;
        er_seg.size = (PAGE_SIZE / size_of::<XhciTrb>()) as u32;
        interrupter0.erstba.update_volatile(|erstba| {
            erstba.set(self.eseg_phys.data() as u64);
        });

        let spb = capability
            .hcsparams2
            .read_volatile()
            .max_scratchpad_buffers();
        if spb != 0 {
            let frame_count = FrameCount::new(size_of::<u64>() * spb as usize);
            let spba_phys = unsafe { FRAME_ALLOCATOR.lock().allocate(frame_count) }.unwrap();
            let frame_count = FrameCount::new(PAGE_SIZE * spb as usize);
            let pad = unsafe { FRAME_ALLOCATOR.lock().allocate(frame_count) }.unwrap();

            let spba_virt = unsafe { CurrentRmmArch::phys_to_virt(spba_phys) };

            for i in 0..spb as usize {
                unsafe {
                    (spba_virt.data() as *mut u64)
                        .add(i)
                        .write_volatile(pad.data() as u64 + i as u64 * PAGE_SIZE as u64)
                };
            }

            unsafe {
                (dcbaa_virt.data() as *mut u64).write_volatile(spba_phys.data() as u64);
            }
        }

        operational.usbcmd.update_volatile(|usbcmd| {
            usbcmd.set_run_stop();
        });

        while operational.usbsts.read_volatile().hc_halted() {
            spin_loop();
        }

        CurrentTimeArch::delay(10 * 1000000);

        interrupter0.iman.update_volatile(|iman| {
            iman.set_0_interrupt_pending();
            iman.set_interrupt_enable();
        });
        interrupter0.imod.update_volatile(|imod| {
            imod.set_interrupt_moderation_counter(0);
            imod.set_interrupt_moderation_interval(0);
        });

        operational.usbcmd.update_volatile(|usbcmd| {
            usbcmd.set_interrupter_enable();
        });
    }

    pub fn get_pipe(&mut self, slot: u8, epid: u8) -> Option<ArcXhciPipe> {
        self.pipes.get(&slot)?.get(&epid).cloned()
    }

    pub fn xfer_setup_in(
        &mut self,
        arc_pipe: ArcXhciPipe,
        cmd: &[u8],
        data: Option<&mut [u8]>,
    ) -> u8 {
        let pipe = arc_pipe.read();
        let mut ring = pipe.ring.lock();
        let cmd = unsafe { (cmd.as_ptr() as *const u64).read_unaligned() };
        ring.submit_trb(|trb, cs| trb.setup(cmd, TransferKind::In, cs));

        if let Some(data) = data {
            let len = data.len();
            let addr_unaligned = data.as_mut_ptr() as usize;
            let addr_aligned = align_down(addr_unaligned);
            let offset = addr_unaligned - addr_aligned;
            let addr = VirtualAddress::new(addr_aligned);
            let mut frame_allocator = FRAME_ALLOCATOR.lock();
            let page_mapper = unsafe {
                PageMapper::<CurrentRmmArch, _>::current(
                    rmm::TableKind::Kernel,
                    &mut *frame_allocator,
                )
            };
            let (phys, _) = page_mapper.translate(addr).unwrap();
            ring.submit_trb(|trb, cs| trb.data(phys.add(offset).data(), len as u16, true, cs));
        }

        ring.submit_trb(|trb, cs| trb.status(0, false, true, false, false, cs));

        self.doorbell(pipe.slot, pipe.epid);
        drop(ring);
        drop(pipe);
        self.event_wait_pipe(arc_pipe.clone())
    }

    pub fn xfer_setup_out(&mut self, arc_pipe: ArcXhciPipe, cmd: &[u8], data: Option<&[u8]>) -> u8 {
        let pipe = arc_pipe.read();
        let mut ring = pipe.ring.lock();
        let cmd = unsafe { (cmd.as_ptr() as *const u64).read_unaligned() };
        ring.submit_trb(|trb, cs| trb.setup(cmd, TransferKind::Out, cs));

        if let Some(data) = data {
            let len = data.len();
            let addr_unaligned = data.as_ptr() as usize;
            let addr_aligned = align_down(addr_unaligned);
            let offset = addr_unaligned - addr_aligned;
            let addr = VirtualAddress::new(addr_aligned);
            let mut frame_allocator = FRAME_ALLOCATOR.lock();
            let page_mapper = unsafe {
                PageMapper::<CurrentRmmArch, _>::current(
                    rmm::TableKind::Kernel,
                    &mut *frame_allocator,
                )
            };
            let (phys, _) = page_mapper.translate(addr).unwrap();
            ring.submit_trb(|trb, cs| trb.data(phys.add(offset).data(), len as u16, false, cs));
        }

        ring.submit_trb(|trb, cs| trb.status(0, true, true, false, false, cs));

        self.doorbell(pipe.slot, pipe.epid);
        drop(ring);
        drop(pipe);
        self.event_wait_pipe(arc_pipe.clone())
    }

    fn submit_trb_split(
        &mut self,
        arc_pipe: ArcXhciPipe,
        phys: PhysicalAddress,
        len: usize,
        _dir_in: bool,
    ) {
        let pipe = arc_pipe.read();
        let mut ring = pipe.ring.lock();

        let mut offset = 0;
        let chunk_size = pipe.maxpacket as usize;

        let mut td_size = ((len + chunk_size - 1) / chunk_size).min(0x1F);

        while offset < len {
            let current_phys = phys.add(offset);

            let ioc = offset + chunk_size >= len;

            ring.submit_trb(|trb, cs| {
                trb.normal(
                    current_phys.data() as u64,
                    chunk_size as u32,
                    cs,
                    td_size as u8,
                    0,
                    false,
                    false,
                    !ioc,
                    ioc,
                    false,
                    false,
                )
            });

            offset += chunk_size;
            td_size -= 1;
        }
    }

    pub fn xfer_normal_in(&mut self, arc_pipe: ArcXhciPipe, data: &mut [u8]) -> u8 {
        let len = data.len();
        let addr_unaligned = data.as_mut_ptr() as usize;
        let addr_aligned = align_down(addr_unaligned);
        let offset = addr_unaligned - addr_aligned;
        let addr = VirtualAddress::new(addr_aligned);
        let mut frame_allocator = FRAME_ALLOCATOR.lock();
        let page_mapper = unsafe {
            PageMapper::<CurrentRmmArch, _>::current(rmm::TableKind::Kernel, &mut *frame_allocator)
        };
        let (phys, _) = page_mapper.translate(addr).unwrap();
        self.submit_trb_split(arc_pipe.clone(), phys, len, true);

        let pipe = arc_pipe.read();
        self.doorbell(pipe.slot, pipe.epid);
        drop(pipe);
        self.event_wait_pipe(arc_pipe.clone())
    }

    pub fn xfer_normal_out(&mut self, arc_pipe: ArcXhciPipe, data: &[u8]) -> u8 {
        let len = data.len();
        let addr_unaligned = data.as_ptr() as usize;
        let addr_aligned = align_down(addr_unaligned);
        let offset = addr_unaligned - addr_aligned;
        let addr = VirtualAddress::new(addr_aligned);
        let mut frame_allocator = FRAME_ALLOCATOR.lock();
        let page_mapper = unsafe {
            PageMapper::<CurrentRmmArch, _>::current(rmm::TableKind::Kernel, &mut *frame_allocator)
        };
        let (phys, _) = page_mapper.translate(addr).unwrap();
        self.submit_trb_split(arc_pipe.clone(), phys, len, true);

        let pipe = arc_pipe.read();
        self.doorbell(pipe.slot, pipe.epid);
        drop(pipe);
        self.event_wait_pipe(arc_pipe.clone())
    }

    pub fn configure_device(&mut self, device: ArcUsbDevice, speed: XhciSpeed) {
        let ctrlsize = match speed {
            XhciSpeed::Full => 8,
            XhciSpeed::Low => 8,
            XhciSpeed::High => 64,
            XhciSpeed::Super => 128,
            _ => 8,
        };
        let mut fake_epdesc = UsbEndpointDescriptor::default();
        fake_epdesc.max_packet_size = ctrlsize;
        fake_epdesc.attr = 0;

        if let Some(ctrl_pipe) = self.alloc_pipe(device.clone(), fake_epdesc) {
            device.write().ctrl_pipe = ctrl_pipe.clone();

            let mut request = UsbCtrlRequest::default();
            request.req_type = 0x80 | (0x00 << 5) | 0x00;
            request.req = 0x06;
            request.value = 0x01 << 8;
            request.index = 0;
            request.length = 8;

            let mut descriptor = UsbDeviceDescriptor::default();

            self.read_pipe(ctrl_pipe.clone(), Some(&request), Some(&mut descriptor));

            let mut maxpacket = descriptor.max_packet_size_0 as u16;
            if descriptor.bcd_usb >= 0x0300 {
                maxpacket = 1 << maxpacket;
            }
            let mut fake_epdesc = UsbEndpointDescriptor::default();
            fake_epdesc.max_packet_size = maxpacket;
            fake_epdesc.attr = 0;

            if let Some(ctrl_pipe) =
                self.realloc_pipe(device.clone(), Some(ctrl_pipe.clone()), Some(fake_epdesc))
            {
                device.write().ctrl_pipe = ctrl_pipe.clone();

                let mut request = UsbCtrlRequest::default();
                request.req_type = 0x80 | (0x00 << 5) | 0x00;
                request.req = 0x06;
                request.value = 0x01 << 8;
                request.index = 0;
                request.length = maxpacket;

                self.read_pipe(ctrl_pipe.clone(), Some(&request), Some(&mut descriptor));

                device.write().product_id = descriptor.id_product;
                device.write().vendor_id = descriptor.id_vendor;

                let mut config = UsbConfigDescriptor::default();

                request.req_type = 0x80 | (0x00 << 5) | 0x00;
                request.req = 0x06;
                request.value = 0x02 << 8;
                request.index = 0;
                request.length = size_of::<UsbConfigDescriptor>() as u16;

                self.read_pipe(ctrl_pipe.clone(), Some(&request), Some(&mut config));

                let mut buffer = Vec::new();
                buffer.resize(config.total_length as usize, 0);
                request.length = config.total_length;
                self.read_pipe(ctrl_pipe.clone(), Some(&request), Some(&mut buffer));

                let mut interface_descriptor_start = unsafe {
                    buffer.as_ptr().byte_add(config.length as usize)
                        as *const UsbInterfaceDescriptor
                };

                let mut num_iface = config.num_interfaces;

                let mut ptr = 0;
                while num_iface != 0 && ptr < config.total_length as usize {
                    let interface_descriptor =
                        unsafe { interface_descriptor_start.as_ref_unchecked() };
                    let interface_descriptor_len = interface_descriptor.length as usize;

                    if interface_descriptor.desc_type == 0x04 {
                        num_iface -= 1;
                        device.write().ifaces.push(interface_descriptor.clone());
                    }

                    interface_descriptor_start =
                        unsafe { interface_descriptor_start.byte_add(interface_descriptor_len) };
                    ptr += interface_descriptor_len;
                }

                drop(buffer);

                CurrentTimeArch::delay(100 * 1000000);

                request.req_type = 0x00 | (0x00 << 5) | 0x00;
                request.req = 0x09;
                request.value = config.configuration_value as u16;
                request.index = 0;
                request.length = 0;
                self.write_pipe(ctrl_pipe.clone(), Some(&request), None);

                USB_DEVICES.lock().push(device);
            }
        }
    }

    pub fn enumerate(&mut self) -> Result<ArcUsbHub, &str> {
        CurrentTimeArch::delay(20 * 1000000);
        let usbhub = Arc::new(RwLock::new(UsbHub {
            usbdev: None,
            port: 0,
            portcount: self.max_ports,
        }));
        'for_loop: for port in 0..self.max_ports {
            if self
                .registers
                .port_register_set
                .read_volatile_at(port as usize)
                .portsc
                .current_connect_status()
            {
                info!("Found connected USB device at port {}", port);

                CurrentTimeArch::delay(100 * 1000000);
                let pls = self
                    .registers
                    .port_register_set
                    .read_volatile_at(port as usize)
                    .portsc
                    .port_link_state();
                match pls {
                    0 => {}
                    7 => {
                        self.registers.port_register_set.update_volatile_at(
                            port as usize,
                            |port| {
                                port.portsc.set_port_reset();
                            },
                        );
                    }
                    _ => panic!("Invalid USB port link state"),
                }

                loop {
                    CurrentTimeArch::delay(1 * 1000000);

                    if !self
                        .registers
                        .port_register_set
                        .read_volatile_at(port as usize)
                        .portsc
                        .current_connect_status()
                    {
                        continue 'for_loop;
                    }
                    if self
                        .registers
                        .port_register_set
                        .read_volatile_at(port as usize)
                        .portsc
                        .port_enabled_disabled()
                    {
                        break;
                    }
                }

                info!("USB port {} enabled", port);

                CurrentTimeArch::delay(10 * 1000000);
                let xhci_speed = match self
                    .registers
                    .port_register_set
                    .read_volatile_at(port as usize)
                    .portsc
                    .port_speed()
                {
                    1 => XhciSpeed::Full,
                    2 => XhciSpeed::Low,
                    3 => XhciSpeed::High,
                    4 => XhciSpeed::Super,
                    _ => XhciSpeed::Unknown,
                };

                CurrentTimeArch::delay(10 * 1000000);

                let device = Arc::new(RwLock::new(UsbDevice {
                    hub: usbhub.clone(),
                    ctrl_pipe: Arc::new(RwLock::new(Default::default())),
                    slot: 0,
                    port: 0,
                    product_id: 0,
                    vendor_id: 0,
                    speed: UsbDeviceSpeed::Unknown,
                    devaddr: 0,
                    ifaces: Vec::new(),
                }));

                self.configure_device(device.clone(), xhci_speed);
            }
        }
        Ok(usbhub)
    }

    pub fn process_events(&mut self) -> bool {
        let mut event_ring = self.event_ring.lock();
        let mut idx = event_ring.idx;
        let trb = event_ring.trbs()[idx].clone();
        if trb.cycle() != event_ring.cs {
            return false;
        }

        let evt_type = trb.trb_type();
        let evt_cc = trb.completion_code();

        match evt_type {
            32 => {
                let slotid = trb.event_slot();
                let epid = trb.endpoint_id();

                if let Some(device) = self.pipes.get(&slotid) {
                    if let Some(endpoint) = device.get(&epid) {
                        let endpoint = endpoint.write();
                        let mut ring = endpoint.ring.lock();
                        ring.result.set(trb.read_data(), trb.status, trb.control);
                        ring.event_idx = (trb.read_data() as usize - ring.ring_paddr.data())
                            / size_of::<XhciTrb>()
                            + 1;
                    }
                }
            }

            33 => {
                let mut cmd_ring = self.cmd_ring.lock();
                cmd_ring
                    .result
                    .set(trb.read_data(), trb.status, trb.control);
                cmd_ring.event_idx = (trb.read_data() as usize - cmd_ring.ring_paddr.data())
                    / size_of::<XhciTrb>()
                    + 1;
            }
            _ => {}
        }

        idx += 1;
        if idx >= (PAGE_SIZE / size_of::<XhciTrb>() - 1) {
            idx = 0;
            event_ring.cs = !event_ring.cs;
        }

        event_ring.idx = idx;

        let interrupter = &mut self.registers.interrupter_register_set;
        let mut interrupt0 = interrupter.interrupter_mut(0);

        let addr = event_ring.ring_paddr.add(idx * size_of::<XhciTrb>());
        interrupt0.erdp.update_volatile(|erdp| {
            erdp.set_event_ring_dequeue_pointer(addr.data() as u64);
            erdp.set_0_event_handler_busy();
        });

        return true;
    }

    pub fn doorbell(&mut self, slotid: u8, epid: u8) {
        let doorbell = &mut self.registers.doorbell;
        doorbell.update_volatile_at(slotid as usize, |db| {
            db.set_doorbell_target(epid);
        });
    }

    pub fn event_wait_pipe(&mut self, pipe: ArcXhciPipe) -> u8 {
        loop {
            let pipe = pipe.read();
            let ring = pipe.ring.lock();
            let idx = ring.idx;
            let eidx = ring.event_idx;
            drop(ring);
            drop(pipe);
            if idx == eidx {
                break;
            }
            while self.process_events() {
                spin_loop();
            }
        }
        (pipe.read().ring.lock().result.status >> 24) as u8
    }

    pub fn event_wait_cmd(&mut self) -> u8 {
        loop {
            let ring = self.cmd_ring.lock();
            let idx = ring.idx;
            let eidx = ring.event_idx;
            drop(ring);
            if idx == eidx {
                break;
            }
            while self.process_events() {
                spin_loop();
            }
        }
        (self.cmd_ring.lock().result.status >> 24) as u8
    }

    pub fn enable_slot(&mut self) -> u8 {
        self.cmd_ring
            .lock()
            .submit_trb(|trb, cs| trb.enable_slot(0, cs));
        self.doorbell(0, 0);
        self.event_wait_cmd()
    }

    pub fn address_device(&mut self, slotid: u8, inctx: PhysicalAddress) -> u8 {
        self.cmd_ring
            .lock()
            .submit_trb(|trb, cs| trb.address_device(slotid, inctx.data() as usize, false, cs));
        self.doorbell(0, 0);
        self.event_wait_cmd()
    }

    pub fn evaluate_context(&mut self, slotid: u8, inctx: PhysicalAddress) -> u8 {
        self.cmd_ring
            .lock()
            .submit_trb(|trb, cs| trb.evaluate_context(slotid, inctx.data() as usize, false, cs));
        self.doorbell(0, 0);
        self.event_wait_cmd()
    }

    pub fn alloc_inctx(
        &mut self,
        usbdev: ArcUsbDevice,
        max_epid: u8,
    ) -> (VirtualAddress, PhysicalAddress, FrameCount) {
        let device = usbdev.read();

        let size = (size_of::<XhciInctx>() * 33) << self.context64_shift;
        let aligned_size = align_up(size);
        let frame_count = FrameCount::new(aligned_size / PAGE_SIZE);
        let inctx_phys = unsafe { FRAME_ALLOCATOR.lock().allocate(frame_count) }.unwrap();
        let inctx_virt = unsafe { CurrentRmmArch::phys_to_virt(inctx_phys) };
        unsafe {
            core::ptr::write_bytes(inctx_virt.data() as *mut u8, 0, size);
        }

        let slotctx_virt = inctx_virt.add(size_of::<XhciInctx>() << self.context64_shift);
        let slotctx = unsafe { (slotctx_virt.data() as *mut XhciSlotctx).as_mut_unchecked() };
        slotctx.ctx[0] |= (max_epid as u32) << 27;
        slotctx.ctx[0] |= (usbspeed_to_xhcispeed(device.speed) as u32) << 20;

        if let Some(hubdev) = &device.hub.read().usbdev {
            if device.speed == UsbDeviceSpeed::Low || device.speed == UsbDeviceSpeed::Full {
                if hubdev.read().speed == UsbDeviceSpeed::High {
                    slotctx.ctx[2] |= hubdev.read().slot as u32;
                    slotctx.ctx[2] |= ((device.port + 1) as u32) << 8;
                } else {
                    let dcbaa_virt = unsafe { CurrentRmmArch::phys_to_virt(self.dcbaa_phys) };
                    let hubslotctx_phys = PhysicalAddress::new(unsafe {
                        (dcbaa_virt.data() as *const usize)
                            .add(hubdev.read().slot as usize)
                            .read_volatile()
                    });
                    let hubslotctx_virt = unsafe { CurrentRmmArch::phys_to_virt(hubslotctx_phys) };
                    let hubslotctx = unsafe {
                        (hubslotctx_virt.data() as *const XhciSlotctx).as_ref_unchecked()
                    };
                    slotctx.ctx[2] = hubslotctx.ctx[2];
                }
            }

            let mut route = 0;
            let mut usbdev = Some(usbdev.clone());
            loop {
                if let Some(device) = usbdev {
                    route <<= 4;
                    route |= ((device.read().port + 1) & 0xf) as u32;
                    usbdev = device.read().hub.read().usbdev.clone();
                } else {
                    break;
                }
            }
            slotctx.ctx[0] |= route;
        }

        slotctx.ctx[1] |= ((device.port + 1) as u32) << 16;

        (inctx_virt, inctx_phys, frame_count)
    }

    pub fn alloc_pipe(
        &mut self,
        device: ArcUsbDevice,
        epdesc: UsbEndpointDescriptor,
    ) -> Option<ArcUsbPipe> {
        let eptype = epdesc.attr & 0x7f;
        let epid = if epdesc.ep_addr == 0 {
            1
        } else {
            (epdesc.ep_addr & 0xf) * 2 + ((epdesc.ep_addr & 0x80) >> 7)
        };

        let new_ring = XhciRing::new();
        let new_ring_phys = new_ring.ring_paddr.data() as u64;

        let xhci_pipe = Arc::new(RwLock::new(XhciPipe {
            slot: 0,
            epid,
            maxpacket: epdesc.max_packet_size,
            ring: Mutex::new(new_ring),
        }));

        let (inctx_virt, inctx_phys, frame_count) = self.alloc_inctx(device.clone(), epid);
        let inctx = unsafe { (inctx_virt.data() as *mut XhciInctx).as_mut_unchecked() };
        inctx.add |= (1 << 0) | (1 << epid);
        let epctx_virt =
            inctx_virt.add(((epid as usize + 1) * size_of::<XhciInctx>()) << self.context64_shift);
        let epctx = unsafe { (epctx_virt.data() as *mut XhciEpctx).as_mut_unchecked() };
        if eptype == 3 {
            epctx.ctx[0] |= (usb_get_period(device.clone(), epdesc) + 3) << 16;
        }
        epctx.ctx[1] |= (eptype as u32) << 3;
        if ((epdesc.ep_addr & 0x80) == 0x80) || (eptype == 0) {
            epctx.ctx[1] |= 1 << 5;
        }
        epctx.deq_low = new_ring_phys as u32;
        epctx.deq_high = (new_ring_phys >> 32) as u32;
        epctx.deq_low |= 1;
        epctx.length = epdesc.max_packet_size as u32;

        if epid == 1 {
            let dcba_size = (size_of::<XhciSlotctx>() * 32) << self.context64_shift;
            let dcba = unsafe {
                FRAME_ALLOCATOR
                    .lock()
                    .allocate(FrameCount::new(dcba_size / PAGE_SIZE))
            }
            .unwrap();

            device.write().slot = self.enable_slot();

            let dcbaa_virt = unsafe { CurrentRmmArch::phys_to_virt(self.dcbaa_phys) };
            unsafe {
                (dcbaa_virt.data() as *mut u64)
                    .add(device.read().slot as usize)
                    .write_volatile(dcba.data() as u64)
            };

            self.address_device(device.read().slot, inctx_phys);

            self.pipes.insert(device.read().slot, BTreeMap::new());
        }

        let slot = device.read().slot;

        let eps = self.pipes.get_mut(&slot).unwrap();
        xhci_pipe.write().slot = slot;
        eps.insert(epid, xhci_pipe);
        let usbpipe = UsbPipe {
            slot,
            epid,
            eptype,
            maxpacket: epdesc.max_packet_size,
        };

        unsafe { FRAME_ALLOCATOR.lock().free(inctx_phys, frame_count) };

        Some(Arc::new(RwLock::new(usbpipe)))
    }
}

impl UsbHcd for XhciHcd {
    fn realloc_pipe(
        &mut self,
        device: ArcUsbDevice,
        pipe: Option<ArcUsbPipe>,
        epdesc: Option<UsbEndpointDescriptor>,
    ) -> Option<ArcUsbPipe> {
        if let Some(epdesc) = epdesc {
            if let Some(pipe) = pipe {
                let eptype = epdesc.attr & 0x7F;
                let old_maxpacket = pipe.read().maxpacket;
                pipe.write().eptype = eptype;
                pipe.write().maxpacket = epdesc.max_packet_size;
                if (eptype != 0) || (epdesc.max_packet_size == old_maxpacket) {
                    Some(pipe.clone())
                } else {
                    let (inctx_virt, inctx_phys, frame_count) = self.alloc_inctx(device.clone(), 1);
                    let inctx = unsafe { (inctx_virt.data() as *mut XhciInctx).as_mut_unchecked() };
                    inctx.add |= 1 << 1;
                    let epctx_virt =
                        inctx_virt.add((2 * size_of::<XhciInctx>()) << self.context64_shift);
                    let epctx = unsafe { (epctx_virt.data() as *mut XhciEpctx).as_mut_unchecked() };
                    epctx.ctx[1] |= (epdesc.max_packet_size as u32) << 16;

                    self.evaluate_context(pipe.read().slot, inctx_phys);

                    unsafe { FRAME_ALLOCATOR.lock().free(inctx_phys, frame_count) };

                    Some(pipe.clone())
                }
            } else {
                self.alloc_pipe(device, epdesc)
            }
        } else {
            if let Some(pipe) = pipe {
                // free
                None
            } else {
                None
            }
        }
    }

    fn read_pipe(&mut self, arc_pipe: ArcUsbPipe, cmd: Option<&[u8]>, data: Option<&mut [u8]>) {
        let pipe = arc_pipe.read();
        if let Some(xhci_pipe) = self.get_pipe(pipe.slot, pipe.epid) {
            if let Some(cmd) = cmd {
                drop(pipe);
                self.xfer_setup_in(xhci_pipe, cmd, data);
            } else if let Some(data) = data {
                drop(pipe);
                self.xfer_normal_in(xhci_pipe, data);
            }
        } else {
        }
    }
    fn write_pipe(&mut self, arc_pipe: ArcUsbPipe, cmd: Option<&[u8]>, data: Option<&[u8]>) {
        let pipe = arc_pipe.read();
        if let Some(xhci_pipe) = self.get_pipe(pipe.slot, pipe.epid) {
            if let Some(cmd) = cmd {
                drop(pipe);
                self.xfer_setup_out(xhci_pipe, cmd, data);
            } else if let Some(data) = data {
                drop(pipe);
                self.xfer_normal_out(xhci_pipe, data);
            }
        } else {
        }
    }

    fn read_intr_pipe(&mut self, pipe: ArcUsbPipe, data: &mut [u8], cb: fn(u64)) {}
    fn write_intr_pipe(&mut self, pipe: ArcUsbPipe, data: &[u8], cb: fn(u64)) {}
}

pub static XHCI_HCDS: Mutex<Vec<Arc<RwLock<XhciHcd>>>> = Mutex::new(Vec::new());

pub fn init() {
    if let Some(pci_devices) = PCI_DEVICES.lock().as_ref() {
        for device in pci_devices.iter().filter(|device| {
            device.class == 0x0C && device.sub_class == 0x03 && device.interface == 0x30
        }) {
            let bar0 = device.bars[0].unwrap();
            let (addr, size) = bar0.unwrap_mem();
            let mut xhci_hcd = XhciHcd::new(PhysicalAddress::new(addr));
            xhci_hcd.enumerate().unwrap();
            XHCI_HCDS.lock().push(Arc::new(RwLock::new(xhci_hcd)));
        }
    }
}
