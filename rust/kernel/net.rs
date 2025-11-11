#![no_std]

use core::cell::UnsafeCell;
use core::ffi::c_uint;
use core::panic::PanicInfo;
use core::ptr;

extern "C" {
    fn panic(msg: *const u8) -> !;
    fn cprintf(fmt: *const u8, ...);
    fn microdelay(us: i32);
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

const REG_EERD: usize      = 0x00014;

const EERD_START: u32      = 1 << 0;
const EERD_DONE: u32       = 1 << 4;
const EERD_ADDR_SHIFT: u32 = 1 << 3;
const EERD_DATA_SHIFT: u32 = 1 << 4;

struct NetState {
    mmio: *mut u8,
    mac: [u8; 6],
    bus: u8,
    slot: u8,
    func: u8,
}

struct NetStateCell(UnsafeCell<NetState>);

impl NetStateCell {
    const fn new(state: NetState) -> Self {
        NetStateCell(UnsafeCell::new(state))
    }

    unsafe fn get(&self) -> &mut NetState {
        &mut *self.0.get()
    }
}

unsafe impl Sync for NetStateCell {}

static STATE: NetStateCell = NetStateCell::new(NetState {
    mmio: ptr::null_mut(),
    mac: [0; 6],
    bus: 0,
    slot: 0,
    func: 0,
});

#[panic_handler]
fn panic_handler(_info: &PanicInfo) -> ! {
    unsafe { panic(b"kernel panic in rust code\0".as_ptr()) }
}

fn cpanic(msg: &[u8]) -> ! {
    unsafe { 
        cprintf(msg.as_ptr());
        panic(b"panic\0".as_ptr())
    }
}

fn print(msg: &[u8]) {
    unsafe { cprintf(msg.as_ptr()) }
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
    unsafe {
        cprintf(
            b"e1000 mac %b:%b:%b:%b:%b:%b\n\0".as_ptr(),
            state.mac[0] as c_uint,
            state.mac[1] as c_uint,
            state.mac[2] as c_uint,
            state.mac[3] as c_uint,
            state.mac[4] as c_uint,
            state.mac[5] as c_uint,
        );
    }
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
}

#[no_mangle]
pub extern "C" fn net_init() {
    net_init_internal();
}