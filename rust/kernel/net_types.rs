use core::cell::UnsafeCell;

pub const NET_RX_DESC_COUNT: usize = 32;
pub const NET_TX_DESC_COUNT: usize = 32;
pub const PGSIZE: usize = 4096;
pub const TX_STATUS_DD: u8 = 1 << 0;

#[repr(C)]
pub struct RxDesc {
    pub addr: u64,
    pub length: u16,
    pub checksum: u16,
    pub status: u8,
    pub errors: u8,
    pub special: u16,
}

#[repr(C)]
pub struct TxDesc {
    pub addr: u64,
    pub length: u16,
    pub cso: u8,
    pub cmd: u8,
    pub status: u8,
    pub css: u8,
    pub special: u16,
}

pub struct NetState {
    pub mmio: *mut u8,
    pub mac: [u8; 6],
    pub bus: u8,
    pub slot: u8,
    pub func: u8,
    pub rx_descs: *mut RxDesc,
    pub rx_buffers: [*mut u8; NET_RX_DESC_COUNT],
    pub rx_cur: u32,
    pub tx_descs: *mut TxDesc,
    pub tx_buffers: [*mut u8; NET_TX_DESC_COUNT],
    pub tx_tail: u32,
    pub initialized: bool,
}

pub struct NetStateCell(UnsafeCell<NetState>);

impl NetStateCell {
    pub const fn new(state: NetState) -> Self {
        NetStateCell(UnsafeCell::new(state))
    }

    pub unsafe fn get(&self) -> &mut NetState {
        &mut *self.0.get()
    }
}

unsafe impl Send for NetStateCell {}
unsafe impl Sync for NetStateCell {}
