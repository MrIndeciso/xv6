#![no_std]
#![no_main]

extern crate userlib;

use userlib::{exit, getmac, println, write};

#[no_mangle]
pub extern "C" fn main() -> i32 {
    let mut mac = [0u8; 6];

    if getmac(&mut mac) < 0 {
        println!("netd: failed to read mac address");
        exit(1);
    }

    println!("netd: mac address");
    print_mac(&mac);
    exit(0)
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
