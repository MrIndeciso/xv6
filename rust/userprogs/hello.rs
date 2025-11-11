#![no_std]
#![no_main]

extern crate userlib;

use userlib::{exit, getpid, println, println_u64, sleep, uptime};

#[no_mangle]
pub extern "C" fn main() -> i32 {
    println!("hello from rust!");
    println!("pid:");
    println_u64(getpid() as u64);
    println!("uptime before sleep:");
    println_u64(uptime() as u64);
    sleep(100);
    println!("uptime after sleep:");
    println_u64(uptime() as u64);
    println!("goodbye from rust userland");
    exit(0)
}
