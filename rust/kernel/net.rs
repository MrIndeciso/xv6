#![no_std]

mod net_types;

use core::ffi::{c_uint, c_void};
use core::mem::size_of;
use core::panic::PanicInfo;
use core::ptr;

use net_types::{
    NetState,
    NetStateCell,
    RxDesc,
    TxDesc,
    NET_RX_DESC_COUNT,
    NET_TX_DESC_COUNT,
    PGSIZE,
    TX_STATUS_DD,
};

extern "C" {
    fn panic(msg: *const u8) -> !;
    fn cprintf(fmt: *const u8, ...);
    fn microdelay(us: i32);
    fn kalloc() -> *mut u8;
    fn wakeup(chan: *mut c_void);
    fn ioapicenable(irq: i32, cpu: i32);
}

macro_rules! kprintf {
    ($fmt:expr $(, $arg:expr )* $(,)?) => {{
        unsafe { cprintf($fmt.as_ptr(), $( $arg ),*) }
    }};
}

unsafe fn inl(port: u16) -> u32 {
    let value: u32;
    core::arch::asm!(
        "inl %dx, %eax",
        in("dx") port,
        lateout("eax") value,
        options(att_syntax, nomem, nostack, preserves_flags)
    );
    value
}

unsafe fn outl(port: u16, data: u32) {
    core::arch::asm!(
        "outl %eax, %dx",
        in("dx") port,
        in("eax") data,
        options(att_syntax, nomem, nostack, preserves_flags)
    );
}

const KERNBASE: u64        = 0xFFFF8000_00000000;

const REG_EERD: usize        = 0x00014;
const REG_TCTL: usize        = 0x00400;
const REG_TIPG: usize        = 0x00410;
const REG_ICR: usize         = 0x00C0;
const REG_IMS: usize         = 0x00D0;
const REG_RCTL: usize        = 0x00100;
const REG_RXDESCLO: usize    = 0x02800;
const REG_RXDESCHI: usize    = 0x02804;
const REG_RXDESCLEN: usize   = 0x02808;
const REG_RXDESCHEAD: usize  = 0x02810;
const REG_RXDESCTAIL: usize  = 0x02818;
const REG_RXDCTL: usize      = 0x02828;
const REG_TXDESCLO: usize    = 0x03800;
const REG_TXDESCHI: usize    = 0x03804;
const REG_TXDESCLEN: usize   = 0x03808;
const REG_TXDESCHEAD: usize  = 0x03810;
const REG_TXDESCTAIL: usize  = 0x03818;
const REG_RAL: usize         = 0x05400;
const REG_RAH: usize         = 0x05404;
const REG_MTA: usize         = 0x05200;
const IRQ_NET: i32           = 11;

const EERD_START: u32        = 1 << 0;
const EERD_DONE: u32         = 1 << 4;
const EERD_ADDR_SHIFT: u32   = 1 << 3;
const EERD_DATA_SHIFT: u32   = 1 << 4;

const RXDCTL_PTHRESH_SHIFT: u32 = 0;
const RXDCTL_HTHRESH_SHIFT: u32 = 8;
const RXDCTL_WTHRESH_SHIFT: u32 = 16;
const RXDCTL_ENABLE: u32        = 1 << 25;
const RCTL_EN: u32          = 1 << 1;
const RCTL_BAM: u32         = 1 << 15;
const RCTL_SECRC: u32       = 1 << 26;
const RCTL_BSIZE_2048: u32  = 0;

const TCTL_EN: u32          = 1 << 1;
const TCTL_PSP: u32         = 1 << 3;
const TCTL_CT_SHIFT: u32    = 4;
const TCTL_COLD_SHIFT: u32  = 12;
const TIPG_DEFAULT: u32     = 0x0060_200A;
const INT_TXDW: u32         = 1 << 0;
const INT_RXT0: u32         = 1 << 7;
const RX_STATUS_DD: u8      = 1 << 0;
const RX_STATUS_EOP: u8     = 1 << 1;
const TX_CMD_EOP: u8        = 1 << 0;
const TX_CMD_IFCS: u8       = 1 << 1;
const TX_CMD_RS: u8         = 1 << 3;
static STATE: NetStateCell = NetStateCell::new(NetState {
    mmio: ptr::null_mut(),
    mac: [0; 6],
    bus: 0,
    slot: 0,
    func: 0,
    rx_descs: ptr::null_mut(),
    rx_buffers: [ptr::null_mut(); NET_RX_DESC_COUNT],
    rx_cur: 0,
    tx_descs: ptr::null_mut(),
    tx_buffers: [ptr::null_mut(); NET_TX_DESC_COUNT],
    tx_tail: 0,
    initialized: false,
});

static mut RX_WAIT_CHAN: u8 = 0;
static mut TX_WAIT_CHAN: u8 = 0;

fn v2p(addr: *mut u8) -> u64 {
    (addr as u64).wrapping_sub(KERNBASE)
}

#[panic_handler]
fn panic_handler(_info: &PanicInfo) -> ! {
    unsafe { panic(b"kernel panic in rust code\0".as_ptr()) }
}

fn cpanic(msg: &[u8]) -> ! {
    kprintf!(msg);
    unsafe { panic(b"panic\0".as_ptr()) }
}

fn print(msg: &[u8]) {
    kprintf!(msg)
}

fn pci_config_read_u32(bus: u8, slot: u8, func: u8, offset: u8) -> u32 {
    let address: u32 = (1 << 31)
        | ((bus as u32) << 16)
        | ((slot as u32) << 11)
        | ((func as u32) << 8)
        | ((offset as u32) & 0xFC);
    unsafe {
        outl(0xCF8, address);
        inl(0xCFC)
    }
}

fn pci_config_write_u32(bus: u8, slot: u8, func: u8, offset: u8, value: u32) {
    let address: u32 = (1 << 31)
        | ((bus as u32) << 16)
        | ((slot as u32) << 11)
        | ((func as u32) << 8)
        | ((offset as u32) & 0xFC);
    unsafe {
        outl(0xCF8, address);
        outl(0xCFC, value);
    }
}

fn pci_config_read_u16(bus: u8, slot: u8, func: u8, offset: u8) -> u16 {
    let value = pci_config_read_u32(bus, slot, func, offset);
    let shift = ((offset & 0x2) as u32) * 8;
    ((value >> shift) & 0xFFFF) as u16
}

fn pci_config_write_u16(bus: u8, slot: u8, func: u8, offset: u8, data: u16) {
    let aligned = offset & !0x3;
    let current = pci_config_read_u32(bus, slot, func, aligned);
    let shift = ((offset & 0x2) as u32) * 8;
    let mask = 0xFFFFu32 << shift;
    let value = (current & !mask) | ((data as u32) << shift);
    pci_config_write_u32(bus, slot, func, aligned, value);
}

fn readreg(state: &NetState, offset: usize) -> u32 {
    unsafe {
        if state.mmio.is_null() {
            cpanic(b"e1000 mmio not mapped\0");
        }
        let addr = state.mmio.add(offset) as *mut u32;
        ptr::read_volatile(addr)
    }
}

fn writereg(state: &NetState, offset: usize, value: u32) {
    unsafe {
        if state.mmio.is_null() {
            cpanic(b"e1000 mmio not mapped\0");
        }
        let addr = state.mmio.add(offset) as *mut u32;
        ptr::write_volatile(addr, value);
    }
}

fn eeprom_read(state: &NetState, offset: u32) -> Option<u16> {
    writereg(state, REG_EERD, (offset << EERD_ADDR_SHIFT) | EERD_START);
    for _ in 0..1000 {
        let value = readreg(state, REG_EERD);
        if (value & EERD_DONE) != 0 {
            let word = ((value >> EERD_DATA_SHIFT) & 0xFFFF) as u16;
            return Some(word);
        }
        unsafe { microdelay(1); }
    }
    None
}

fn find_e1000() -> Option<(u8, u8, u8)> {
    for bus in 0u16..=255 {
        for slot in 0u16..32 {
            let bus_u8 = bus as u8;
            let slot_u8 = slot as u8;
            let vendor_id = pci_config_read_u16(bus_u8, slot_u8, 0, 0);
            if vendor_id == 0xFFFF {
                continue;
            }
            let device_id = pci_config_read_u16(bus_u8, slot_u8, 0, 2);
            if vendor_id == 0x8086 && device_id == 0x100E {
                return Some((bus_u8, slot_u8, 0));
            }
        }
    }
    None
}

fn read_mac_address(state: &mut NetState) {
    let mut words = [0u16; 3];

    for (index, word) in words.iter_mut().enumerate() {
        *word = match eeprom_read(state, index as u32) {
            Some(value) => value,
            None => cpanic(b"eeprom read failed\0"),
        };
    }

    state.mac[0] = (words[0] & 0xFF) as u8;
    state.mac[1] = (words[0] >> 8) as u8;
    state.mac[2] = (words[1] & 0xFF) as u8;
    state.mac[3] = (words[1] >> 8) as u8;
    state.mac[4] = (words[2] & 0xFF) as u8;
    state.mac[5] = (words[2] >> 8) as u8;
}

fn program_mac_filters(state: &NetState) {
    let ral = (state.mac[0] as u32)
        | ((state.mac[1] as u32) << 8)
        | ((state.mac[2] as u32) << 16)
        | ((state.mac[3] as u32) << 24);
    let rah = (state.mac[4] as u32)
        | ((state.mac[5] as u32) << 8)
        | (1 << 31);
    writereg(state, REG_RAL, ral);
    writereg(state, REG_RAH, rah);

    for i in 0..128 {
        writereg(state, REG_MTA + (i * 4) as usize, 0);
    }
}

fn enable_receiver(state: &NetState) {
    let rctl = RCTL_EN | RCTL_BAM | RCTL_SECRC | RCTL_BSIZE_2048;
    writereg(state, REG_RCTL, rctl);
}

fn enable_interrupts(state: &NetState) {
    writereg(state, REG_IMS, INT_RXT0 | INT_TXDW);
}

fn rx_init(state: &mut NetState) {
    unsafe {
        let desc_mem = kalloc();
        if desc_mem.is_null() {
            cpanic(b"net: cannot allocate rx descriptors\0");
        }
        ptr::write_bytes(desc_mem, 0, NET_RX_DESC_COUNT * size_of::<RxDesc>());

        state.rx_descs = desc_mem as *mut RxDesc;
        state.rx_cur = 0;

        for i in 0..NET_RX_DESC_COUNT {
            let buf = kalloc();
            if buf.is_null() {
                cpanic(b"net: cannot allocate rx buffer\0");
            }
            ptr::write_bytes(buf, 0, PGSIZE);
            state.rx_buffers[i] = buf;

            let desc = state.rx_descs.add(i);
            (*desc).addr = v2p(buf);
            (*desc).length = 0;
            (*desc).checksum = 0;
            (*desc).status = 0;
            (*desc).errors = 0;
            (*desc).special = 0;
        }

        let rx_phys = v2p(state.rx_descs as *mut u8);
        writereg(state, REG_RXDESCLO, rx_phys as u32);
        writereg(state, REG_RXDESCHI, (rx_phys >> 32) as u32);
        writereg(
            state,
            REG_RXDESCLEN,
            (NET_RX_DESC_COUNT * size_of::<RxDesc>()) as u32,
        );
        writereg(state, REG_RXDESCHEAD, 0);
        writereg(state, REG_RXDESCTAIL, (NET_RX_DESC_COUNT - 1) as u32);

        let rxdctl = (4 << RXDCTL_PTHRESH_SHIFT)
            | (1 << RXDCTL_HTHRESH_SHIFT)
            | (1 << RXDCTL_WTHRESH_SHIFT)
            | RXDCTL_ENABLE;
        writereg(state, REG_RXDCTL, rxdctl);
    }
}

fn tx_init(state: &mut NetState) {
    unsafe {
        let desc_mem = kalloc();
        if desc_mem.is_null() {
            cpanic(b"net: cannot allocate tx descriptors\0");
        }
        ptr::write_bytes(desc_mem, 0, NET_TX_DESC_COUNT * size_of::<TxDesc>());

        state.tx_descs = desc_mem as *mut TxDesc;
        state.tx_tail = 0;

        for i in 0..NET_TX_DESC_COUNT {
            let buf = kalloc();
            if buf.is_null() {
                cpanic(b"net: cannot allocate tx buffer\0");
            }
            ptr::write_bytes(buf, 0, PGSIZE);
            state.tx_buffers[i] = buf;

            let desc = state.tx_descs.add(i);
            (*desc).addr = v2p(buf);
            (*desc).length = 0;
            (*desc).cso = 0;
            (*desc).cmd = 0;
            (*desc).status = TX_STATUS_DD;
            (*desc).css = 0;
            (*desc).special = 0;
        }

        let tx_phys = v2p(state.tx_descs as *mut u8);
        writereg(state, REG_TXDESCLO, tx_phys as u32);
        writereg(state, REG_TXDESCHI, (tx_phys >> 32) as u32);
        writereg(
            state,
            REG_TXDESCLEN,
            (NET_TX_DESC_COUNT * size_of::<TxDesc>()) as u32,
        );
        writereg(state, REG_TXDESCHEAD, 0);
        writereg(state, REG_TXDESCTAIL, 0);

        let tctl = TCTL_EN
            | TCTL_PSP
            | (0x10 << TCTL_CT_SHIFT)
            | (0x40 << TCTL_COLD_SHIFT);
        writereg(state, REG_TCTL, tctl);
        writereg(state, REG_TIPG, TIPG_DEFAULT);
    }
}

fn setup_mmio(state: &mut NetState, bus: u8, slot: u8, func: u8) {
    let bar0 = pci_config_read_u32(bus, slot, func, 0x10);
    if (bar0 & 0x1) != 0 {
        cpanic(b"unexpected io bar for e1000\0");
    }
    let phys = (bar0 & 0xFFFF_FFF0) as u64;
    if phys == 0 {
        cpanic(b"invalid e1000 bar\0");
    }
    state.mmio = ((phys + KERNBASE) as usize) as *mut u8;
    state.bus = bus;
    state.slot = slot;
    state.func = func;

    let mut command = pci_config_read_u16(bus, slot, func, 0x04);
    command |= 1 << 1; // memory space enable
    command |= 1 << 2; // bus master enable
    pci_config_write_u16(bus, slot, func, 0x04, command);
}

fn print_mac_address(state: &NetState) {
    kprintf!(
        b"e1000 mac %b:%b:%b:%b:%b:%b\n\0",
        state.mac[0] as c_uint,
        state.mac[1] as c_uint,
        state.mac[2] as c_uint,
        state.mac[3] as c_uint,
        state.mac[4] as c_uint,
        state.mac[5] as c_uint,
    );
}

fn net_init_internal() {
    print(b"Initializing network...\n\0");
    let (bus, slot, func) = match find_e1000() {
        Some(info) => info,
        None => cpanic(b"e1000 not detected\0"),
    };

    print(b"e1000 detected\n\0");
    let state = unsafe { STATE.get() };
    print(b"Setting up MMIO...\n\0");
    setup_mmio(state, bus, slot, func);
    print(b"Reading MAC address...\n\0");
    read_mac_address(state);
    print_mac_address(state);
    print(b"Initializing RX ring...\n\0");
    rx_init(state);
    print(b"net: rx ring ready\n\0");
    print(b"Initializing TX ring...\n\0");
    tx_init(state);
    print(b"net: tx ring ready\n\0");
    program_mac_filters(state);
    print(b"net: mac filters programmed\n\0");
    enable_receiver(state);
    print(b"net: receiver enabled\n\0");
    enable_interrupts(state);
    print(b"net: interrupts enabled\n\0");
    unsafe {
        ioapicenable(IRQ_NET, 0);
    }
    state.initialized = true;
}

fn rx_wait_channel() -> *mut c_void {
    ptr::addr_of_mut!(RX_WAIT_CHAN).cast::<c_void>()
}

fn tx_wait_channel() -> *mut c_void {
    ptr::addr_of_mut!(TX_WAIT_CHAN).cast::<c_void>()
}

fn recycle_rx_desc(state: &mut NetState, idx: usize) {
    unsafe {
        let desc = &mut *state.rx_descs.add(idx);
        desc.status = 0;
        desc.errors = 0;
        desc.length = 0;
    }
    let next = (idx + 1) % NET_RX_DESC_COUNT;
    state.rx_cur = next as u32;
    let tail = (next + NET_RX_DESC_COUNT - 1) % NET_RX_DESC_COUNT;
    writereg(state, REG_RXDESCTAIL, tail as u32);
}

fn net_wakeup_rx(state: &mut NetState) {
    if state.rx_descs.is_null() {
        return;
    }

    let idx = state.rx_cur as usize;
    let desc = unsafe { &*state.rx_descs.add(idx) };
    if (desc.status & RX_STATUS_DD) != 0 && (desc.status & RX_STATUS_EOP) != 0 {
        unsafe { wakeup(rx_wait_channel()); }
    }
}

fn net_wakeup_tx() {
    unsafe { wakeup(tx_wait_channel()); }
}

#[no_mangle]
pub extern "C" fn net_init() {
    net_init_internal();
}

#[no_mangle]
pub extern "C" fn net_rx(dst: *mut u8, len: u32) -> i32 {
    if dst.is_null() || len == 0 {
        return -1;
    }

    let state = unsafe { STATE.get() };
    if !state.initialized || state.rx_descs.is_null() {
        return -1;
    }

    let idx = state.rx_cur as usize;
    let desc = unsafe { &mut *state.rx_descs.add(idx) };
    if (desc.status & RX_STATUS_DD) == 0 || (desc.status & RX_STATUS_EOP) == 0 {
        return 0;
    }

    if desc.errors != 0 {
        kprintf!(
            b"net: rx descriptor %d errors %d\n\0",
            idx as c_uint,
            desc.errors as c_uint,
        );
        recycle_rx_desc(state, idx);
        return -1;
    }

    let pkt_len = desc.length as usize;
    let max_copy = len as usize;
    if pkt_len == 0 || pkt_len > PGSIZE || pkt_len > max_copy {
        recycle_rx_desc(state, idx);
        return -1;
    }

    let buf = state.rx_buffers[idx];
    if buf.is_null() {
        recycle_rx_desc(state, idx);
        return -1;
    }

    unsafe {
        ptr::copy_nonoverlapping(buf, dst, pkt_len);
    }
    kprintf!(
        b"net: rx packet idx %d len %d\n\0",
        idx as c_uint,
        pkt_len as c_uint,
    );

    recycle_rx_desc(state, idx);
    pkt_len as i32
}

#[no_mangle]
pub extern "C" fn net_tx(src: *const u8, len: u32) -> i32 {
    if src.is_null() || len == 0 {
        return -1;
    }

    if (len as usize) > PGSIZE {
        return -1;
    }

    let state = unsafe { STATE.get() };
    if !state.initialized || state.tx_descs.is_null() {
        return -1;
    }

    let idx = state.tx_tail as usize;
    let desc = unsafe { &mut *state.tx_descs.add(idx) };
    if (desc.status & TX_STATUS_DD) == 0 {
        kprintf!(b"net: tx descriptor %d busy\n\0", idx as c_uint);
        return 0;
    }

    let buf = state.tx_buffers[idx];
    if buf.is_null() {
        return -1;
    }

    let copy_len = len as usize;
    unsafe {
        ptr::copy_nonoverlapping(src, buf, copy_len);
    }

    kprintf!(
        b"net: tx packet idx %d len %d\n\0",
        idx as c_uint,
        copy_len as c_uint,
    );

    desc.length = copy_len as u16;
    desc.cso = 0;
    desc.cmd = TX_CMD_EOP | TX_CMD_IFCS | TX_CMD_RS;
    desc.status = 0;
    desc.css = 0;
    desc.special = 0;

    let next = (idx + 1) % NET_TX_DESC_COUNT;
    state.tx_tail = next as u32;
    writereg(state, REG_TXDESCTAIL, next as u32);

    copy_len as i32
}

#[no_mangle]
pub extern "C" fn net_rx_chan() -> *mut c_void {
    rx_wait_channel()
}

#[no_mangle]
pub extern "C" fn net_tx_chan() -> *mut c_void {
    tx_wait_channel()
}

#[no_mangle]
pub extern "C" fn netintr() {
    print(b"net interrupt received\n\0");

    let state = unsafe { STATE.get() };
    if !state.initialized || state.mmio.is_null() {
        return;
    }

    let icr = readreg(state, REG_ICR);
    if icr == 0 {
        return;
    }
    kprintf!(b"net: interrupt icr=%x\n\0", icr as c_uint);

    if (icr & INT_RXT0) != 0 {
        net_wakeup_rx(state);
    }

    if (icr & INT_TXDW) != 0 {
        net_wakeup_tx();
    }
}

#[no_mangle]
pub extern "C" fn net_get_mac(dst: *mut u8, len: usize) -> i32 {
    if dst.is_null() || len < 6 {
        return -1;
    }
    unsafe {
        let state = STATE.get();
        ptr::copy_nonoverlapping(state.mac.as_ptr(), dst, 6);
    }
    0
}