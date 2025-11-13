#![no_std]

use core::panic::PanicInfo;

// System call numbers
const SYS_FORK: u64 = 1;
const SYS_EXIT: u64 = 2;
const SYS_WAIT: u64 = 3;
const SYS_PIPE: u64 = 4;
const SYS_READ: u64 = 5;
const SYS_KILL: u64 = 6;
// const SYS_EXEC: u64 = 7;
const SYS_FSTAT: u64 = 8;
const SYS_CHDIR: u64 = 9;
const SYS_DUP: u64 = 10;
const SYS_GETPID: u64 = 11;
const SYS_SBRK: u64 = 12;
const SYS_SLEEP: u64 = 13;
const SYS_UPTIME: u64 = 14;
const SYS_OPEN: u64 = 15;
const SYS_WRITE: u64 = 16;
const SYS_MKNOD: u64 = 17;
const SYS_UNLINK: u64 = 18;
const SYS_LINK: u64 = 19;
const SYS_MKDIR: u64 = 20;
const SYS_CLOSE: u64 = 21;
const SYS_GETMAC: u64 = 22;

// File modes
pub const O_RDONLY: i32 = 0x000;
pub const O_WRONLY: i32 = 0x001;
pub const O_RDWR: i32 = 0x002;
pub const O_CREATE: i32 = 0x200;

// File types
pub const T_DIR: i16 = 1;
pub const T_FILE: i16 = 2;
pub const T_DEV: i16 = 3;

#[repr(C)]
pub struct Stat {
    pub file_type: i16,
    pub dev: i32,
    pub ino: u32,
    pub nlink: i16,
    pub size: u32,
}

// Syscall wrapper - unsafe internally, but syscalls themselves are safe
#[inline]
unsafe fn syscall(num: u64, a1: u64, a2: u64, a3: u64) -> i64 {
    let ret: i64;
    core::arch::asm!(
        "mov r10, rcx",
        "syscall",
        in("rax") num,
        in("rdi") a1,
        in("rsi") a2,
        in("rdx") a3,
        lateout("rax") ret,
        out("rcx") _,
        out("r10") _,
        out("r11") _,
    );
    ret
}

pub fn fork() -> i32 {
    unsafe { syscall(SYS_FORK, 0, 0, 0) as i32 }
}

pub fn exit(status: i32) -> ! {
    unsafe { syscall(SYS_EXIT, status as u64, 0, 0) };
    loop {}
}

pub fn wait() -> i32 {
    unsafe { syscall(SYS_WAIT, 0, 0, 0) as i32 }
}

pub fn pipe(fds: &mut [i32; 2]) -> i32 {
    unsafe { syscall(SYS_PIPE, fds.as_mut_ptr() as u64, 0, 0) as i32 }
}

pub fn read(fd: i32, buf: &mut [u8]) -> i32 {
    unsafe { syscall(SYS_READ, fd as u64, buf.as_mut_ptr() as u64, buf.len() as u64) as i32 }
}

pub fn write(fd: i32, buf: &[u8]) -> i32 {
    unsafe { syscall(SYS_WRITE, fd as u64, buf.as_ptr() as u64, buf.len() as u64) as i32 }
}

pub fn close(fd: i32) -> i32 {
    unsafe { syscall(SYS_CLOSE, fd as u64, 0, 0) as i32 }
}

pub fn kill(pid: i32) -> i32 {
    unsafe { syscall(SYS_KILL, pid as u64, 0, 0) as i32 }
}

pub fn open(path: &str, mode: i32) -> i32 {
    unsafe { syscall(SYS_OPEN, path.as_ptr() as u64, mode as u64, 0) as i32 }
}

pub fn mknod(path: &str, major: i16, minor: i16) -> i32 {
    unsafe { syscall(SYS_MKNOD, path.as_ptr() as u64, major as u64, minor as u64) as i32 }
}

pub fn unlink(path: &str) -> i32 {
    unsafe { syscall(SYS_UNLINK, path.as_ptr() as u64, 0, 0) as i32 }
}

pub fn fstat(fd: i32, st: &mut Stat) -> i32 {
    unsafe { syscall(SYS_FSTAT, fd as u64, st as *mut Stat as u64, 0) as i32 }
}

pub fn link(old: &str, new: &str) -> i32 {
    unsafe { syscall(SYS_LINK, old.as_ptr() as u64, new.as_ptr() as u64, 0) as i32 }
}

pub fn mkdir(path: &str) -> i32 {
    unsafe { syscall(SYS_MKDIR, path.as_ptr() as u64, 0, 0) as i32 }
}

pub fn chdir(path: &str) -> i32 {
    unsafe { syscall(SYS_CHDIR, path.as_ptr() as u64, 0, 0) as i32 }
}

pub fn dup(fd: i32) -> i32 {
    unsafe { syscall(SYS_DUP, fd as u64, 0, 0) as i32 }
}

pub fn getpid() -> i32 {
    unsafe { syscall(SYS_GETPID, 0, 0, 0) as i32 }
}

pub fn sbrk(n: i32) -> *mut u8 {
    unsafe { syscall(SYS_SBRK, n as u64, 0, 0) as *mut u8 }
}

pub fn sleep(n: u32) -> i32 {
    unsafe { syscall(SYS_SLEEP, n as u64, 0, 0) as i32 }
}

pub fn uptime() -> i32 {
    unsafe { syscall(SYS_UPTIME, 0, 0, 0) as i32 }
}

pub fn getmac(buf: &mut [u8]) -> i32 {
    if buf.len() < 6 {
        return -1;
    }
    unsafe { syscall(SYS_GETMAC, buf.as_mut_ptr() as u64, buf.len() as u64, 0) as i32 }
}

pub fn print(s: &str) {
    write(1, s.as_bytes());
}

pub fn print_str(s: &str) {
    write(1, s.as_bytes());
}

pub fn println_u64(n: u64) {
    let mut buf = [0u8; 20];
    let mut i = 0;
    let mut num = n;

    if num == 0 {
        buf[0] = b'0';
        i = 1;
    } else {
        let mut temp = [0u8; 20];
        let mut j = 0;
        while num > 0 {
            temp[j] = (num % 10) as u8 + b'0';
            num /= 10;
            j += 1;
        }
        while j > 0 {
            j -= 1;
            buf[i] = temp[j];
            i += 1;
        }
    }

    buf[i] = b'\n';
    write(1, &buf[..=i]);
}

pub fn println_i32(n: i32) {
    if n < 0 {
        write(1, b"-");
        println_u64((-n) as u64);
    } else {
        println_u64(n as u64);
    }
}

#[macro_export]
macro_rules! println {
    () => {{
        $crate::print("\n");
    }};
    ($($arg:tt)*) => {{
        $crate::print(concat!($($arg)*, "\n"));
    }};
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{
        $crate::print_str(concat!($($arg)*));
    }};
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    print_str("PANIC\n");
    exit(1);
}
