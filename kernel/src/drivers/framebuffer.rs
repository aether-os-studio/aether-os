use alloc::vec::Vec;
use limine::request::FramebufferRequest;
use spin::Mutex;

#[allow(dead_code)]
#[derive(Clone, Copy)]
pub struct FrameBuffer {
    pub address: *mut (),
    pub width: usize,
    pub height: usize,
    pub bpp: usize,
    pub pitch: usize,
    pub red_mask_size: usize,
    pub red_mask_shift: usize,
    pub green_mask_size: usize,
    pub green_mask_shift: usize,
    pub blue_mask_size: usize,
    pub blue_mask_shift: usize,
}

unsafe impl Send for FrameBuffer {}

pub static FRAME_BUFFERS: Mutex<Vec<FrameBuffer>> = Mutex::new(Vec::new());

#[used]
#[unsafe(link_section = ".requests")]
static FB_REQUEST: FramebufferRequest = FramebufferRequest::new();

pub fn init() {
    if let Some(fb_response) = FB_REQUEST.get_response() {
        for fb in fb_response.framebuffers() {
            let afb = FrameBuffer {
                address: fb.addr() as *mut (),
                width: fb.width() as usize,
                height: fb.height() as usize,
                bpp: fb.bpp() as usize,
                pitch: fb.pitch() as usize,
                red_mask_size: fb.red_mask_size() as usize,
                red_mask_shift: fb.red_mask_shift() as usize,
                green_mask_size: fb.green_mask_size() as usize,
                green_mask_shift: fb.green_mask_shift() as usize,
                blue_mask_size: fb.blue_mask_size() as usize,
                blue_mask_shift: fb.blue_mask_shift() as usize,
            };
            FRAME_BUFFERS.lock().push(afb);
        }

        crate::drivers::fbterm::init();
    }
}
