#![no_std]
#![no_main]

extern crate userlib;

use userlib::{close, exit, getmac, mknod, open, println, println_u64, read, write, O_RDONLY};

const NETDEV_MAJOR: i16 = 2;
const NETDEV_PATH: &str = "net\0";

#[no_mangle]
pub extern "C" fn main() -> i32 {
    let mut mac = [0u8; 6];

    if getmac(&mut mac) < 0 {
        println!("netd: failed to read mac address");
        exit(1);
    }

    println!("netd: mac address");
    print_mac(&mac);

    let fd = ensure_net_device();
    if fd < 0 {
        println!("netd: unable to open net device");
        exit(1);
    }

    let mut buf = [0u8; 2048];
    let n = read(fd, &mut buf);
    if n > 0 {
        println!("netd: received packet bytes");
        println_u64(n as u64);
    } else if n == 0 {
        println!("netd: no packets available");
    } else {
        println!("netd: read error");
    }

    close(fd);
    exit(0)
}

fn ensure_net_device() -> i32 {
    let mut fd = open(NETDEV_PATH, O_RDONLY);
    if fd >= 0 {
        return fd;
    }
    if mknod(NETDEV_PATH, NETDEV_MAJOR, 0) < 0 {
        return -1;
    }
    fd = open(NETDEV_PATH, O_RDONLY);
    fd
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
