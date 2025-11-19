#![no_std]
#![no_main]

extern crate userlib;

use core::cell::UnsafeCell;
use core::cmp::min;

use userlib::{exit, getmac, mknod, open, print, println, read, sleep, write, O_RDWR};

const NETDEV_MAJOR: i16 = 2;
const NETDEV_PATH: &str = "net\0";

const MY_IP: [u8; 4] = [10, 0, 2, 15];
const MAX_FRAME: usize = 2048;
const MIN_ETH_FRAME: usize = 60;
const ETH_HEADER_LEN: usize = 14;
const IP_HEADER_MIN_LEN: usize = 20;
const ICMP_HEADER_LEN: usize = 8;
const ARP_HEADER_LEN: usize = 28;

const ETH_TYPE_ARP: u16 = 0x0806;
const ETH_TYPE_IP: u16 = 0x0800;
const ARP_REQUEST: u16 = 0x0001;
const ARP_REPLY: u16 = 0x0002;
const IP_PROTO_ICMP: u8 = 1;
const ICMP_ECHO_REQUEST: u8 = 8;
const ICMP_ECHO_REPLY: u8 = 0;

struct FrameBuf(UnsafeCell<[u8; MAX_FRAME]>);

impl FrameBuf {
    const fn new() -> Self {
        Self(UnsafeCell::new([0; MAX_FRAME]))
    }

    unsafe fn get_mut(&self) -> &mut [u8; MAX_FRAME] {
        &mut *self.0.get()
    }
}

unsafe impl Sync for FrameBuf {}

static RX_BUF: FrameBuf = FrameBuf::new();
static TX_BUF: FrameBuf = FrameBuf::new();

#[no_mangle]
pub extern "C" fn main() -> i32 {
    let mut mac = [0u8; 6];
    if getmac(&mut mac) < 0 {
        println!("netd: failed to read mac address");
        exit(1);
    }

    println!("netd: network daemon starting");
    print("netd: mac ");
    print_mac(&mac);
    print("netd: ip ");
    print_ip(&MY_IP);

    let fd = ensure_net_device();
    if fd < 0 {
        println!("netd: unable to open net device");
        exit(1);
    }

    let rx_buf = unsafe { RX_BUF.get_mut() };
    let tx_buf = unsafe { TX_BUF.get_mut() };
    loop {
        let n = read(fd, rx_buf);
        if n <= 0 {
            sleep(1);
            continue;
        }
        let packet_len = min(n as usize, rx_buf.len());
        handle_packet(fd, &rx_buf[..packet_len], &mac, tx_buf);
    }
}

fn ensure_net_device() -> i32 {
    let mut fd = open(NETDEV_PATH, O_RDWR);
    if fd >= 0 {
        return fd;
    }
    if mknod(NETDEV_PATH, NETDEV_MAJOR, 0) < 0 {
        return -1;
    }
    fd = open(NETDEV_PATH, O_RDWR);
    fd
}

fn handle_packet(fd: i32, packet: &[u8], my_mac: &[u8; 6], tx_buf: &mut [u8; MAX_FRAME]) {
    if packet.len() < ETH_HEADER_LEN {
        return;
    }
    let ethertype = read_be_u16(&packet[12..14]);
    if ethertype == ETH_TYPE_ARP {
        handle_arp(fd, packet, my_mac);
    } else if ethertype == ETH_TYPE_IP {
        handle_icmp(fd, packet, my_mac, tx_buf);
    }
}

fn handle_arp(fd: i32, packet: &[u8], my_mac: &[u8; 6]) {
    if packet.len() < ETH_HEADER_LEN + ARP_HEADER_LEN {
        return;
    }

    let arp = &packet[ETH_HEADER_LEN..ETH_HEADER_LEN + ARP_HEADER_LEN];
    let opcode = read_be_u16(&arp[6..8]);
    if opcode != ARP_REQUEST {
        return;
    }

    let target_ip = &arp[24..28];
    if target_ip != MY_IP {
        return;
    }

    println!("netd: received ARP request");

    let mut reply = [0u8; MIN_ETH_FRAME];
    // Ethernet header
    reply[0..6].copy_from_slice(&arp[8..14]); // sender MAC
    reply[6..12].copy_from_slice(my_mac);
    write_be_u16(&mut reply[12..14], ETH_TYPE_ARP);

    // ARP payload
    let reply_arp = &mut reply[ETH_HEADER_LEN..ETH_HEADER_LEN + ARP_HEADER_LEN];
    write_be_u16(&mut reply_arp[0..2], 1); // Ethernet
    write_be_u16(&mut reply_arp[2..4], ETH_TYPE_IP);
    reply_arp[4] = 6;
    reply_arp[5] = 4;
    write_be_u16(&mut reply_arp[6..8], ARP_REPLY);
    reply_arp[8..14].copy_from_slice(my_mac);
    reply_arp[14..18].copy_from_slice(&MY_IP);
    reply_arp[18..24].copy_from_slice(&arp[8..14]);
    reply_arp[24..28].copy_from_slice(&arp[14..18]);

    if write(fd, &reply) >= 0 {
        println!("netd: sent ARP reply");
    } else {
        println!("netd: failed to send ARP reply");
    }
}

fn handle_icmp(fd: i32, packet: &[u8], my_mac: &[u8; 6], tx_buf: &mut [u8; MAX_FRAME]) {
    if packet.len() < ETH_HEADER_LEN + IP_HEADER_MIN_LEN {
        return;
    }

    let ip = &packet[ETH_HEADER_LEN..];
    let ihl = ip[0] & 0x0F;
    if ihl < 5 {
        return;
    }
    let ip_header_len = (ihl as usize) * 4;
    if packet.len() < ETH_HEADER_LEN + ip_header_len {
        return;
    }

    let dest_ip = &ip[16..20];
    if dest_ip != MY_IP {
        return;
    }

    let protocol = ip[9];
    if protocol != IP_PROTO_ICMP {
        return;
    }

    let total_len = read_be_u16(&ip[2..4]) as usize;
    if total_len < ip_header_len + ICMP_HEADER_LEN {
        return;
    }

    let frame_len = ETH_HEADER_LEN + total_len;
    if frame_len > packet.len() || frame_len > MAX_FRAME {
        return;
    }

    let icmp_offset = ETH_HEADER_LEN + ip_header_len;
    let icmp_len = total_len - ip_header_len;
    if icmp_len < ICMP_HEADER_LEN {
        return;
    }

    let icmp = &packet[icmp_offset..icmp_offset + icmp_len];
    if icmp[0] != ICMP_ECHO_REQUEST {
        return;
    }

    println!("netd: received ICMP echo request");

    let reply = &mut tx_buf[..];
    reply[..frame_len].copy_from_slice(&packet[..frame_len]);

    // Ethernet swap
    reply[0..6].copy_from_slice(&packet[6..12]);
    reply[6..12].copy_from_slice(my_mac);

    // IP header adjustments
    let reply_ip = &mut reply[ETH_HEADER_LEN..ETH_HEADER_LEN + ip_header_len];
    reply_ip[12..16].copy_from_slice(&MY_IP);
    reply_ip[16..20].copy_from_slice(&ip[12..16]);
    reply_ip[10] = 0;
    reply_ip[11] = 0;
    let ip_csum = ip_checksum(&reply_ip[..ip_header_len]);
    write_be_u16(&mut reply_ip[10..12], ip_csum);

    // ICMP adjustments
    let reply_icmp = &mut reply[icmp_offset..icmp_offset + icmp_len];
    reply_icmp[0] = ICMP_ECHO_REPLY;
    reply_icmp[2] = 0;
    reply_icmp[3] = 0;
    let icmp_csum = ip_checksum(reply_icmp);
    write_be_u16(&mut reply_icmp[2..4], icmp_csum);

    let send_len = frame_len.max(MIN_ETH_FRAME);
    if send_len > frame_len {
        reply[frame_len..send_len].fill(0);
    }

    if write(fd, &reply[..send_len]) >= 0 {
        println!("netd: sent ICMP echo reply");
    } else {
        println!("netd: failed to send ICMP reply");
    }
}

fn read_be_u16(bytes: &[u8]) -> u16 {
    ((bytes[0] as u16) << 8) | (bytes[1] as u16)
}

fn write_be_u16(dst: &mut [u8], value: u16) {
    dst[0] = (value >> 8) as u8;
    dst[1] = (value & 0xFF) as u8;
}

fn ip_checksum(data: &[u8]) -> u16 {
    let mut sum: u32 = 0;
    let mut chunks = data.chunks_exact(2);
    for chunk in &mut chunks {
        sum += read_be_u16(chunk) as u32;
    }
    if let Some(&byte) = chunks.remainder().first() {
        sum += (byte as u32) << 8;
    }
    while (sum >> 16) != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    !(sum as u16)
}

fn print_mac(mac: &[u8; 6]) {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut buf = [0u8; 18];
    let mut idx = 0;
    for (i, byte) in mac.iter().enumerate() {
        buf[idx] = HEX[(byte >> 4) as usize];
        idx += 1;
        buf[idx] = HEX[(byte & 0x0F) as usize];
        idx += 1;
        if i != mac.len() - 1 {
            buf[idx] = b':';
            idx += 1;
        }
    }
    buf[idx] = b'\n';
    idx += 1;
    let _ = write(1, &buf[..idx]);
}

fn print_ip(ip: &[u8; 4]) {
    let mut buf = [0u8; 16];
    let mut idx = 0;
    for (i, octet) in ip.iter().enumerate() {
        idx += write_decimal(&mut buf[idx..], *octet);
        if i != ip.len() - 1 {
            buf[idx] = b'.';
            idx += 1;
        }
    }
    buf[idx] = b'\n';
    idx += 1;
    let _ = write(1, &buf[..idx]);
}

fn write_decimal(buf: &mut [u8], value: u8) -> usize {
    if value == 0 {
        buf[0] = b'0';
        return 1;
    }
    let mut tmp = [0u8; 3];
    let mut n = value;
    let mut digits = 0;
    while n > 0 {
        tmp[digits] = b'0' + (n % 10);
        n /= 10;
        digits += 1;
    }
    for i in 0..digits {
        buf[i] = tmp[digits - 1 - i];
    }
    digits
}
