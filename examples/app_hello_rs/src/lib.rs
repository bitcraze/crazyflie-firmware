#![no_std]

use panic_halt as _;

extern "C" {
    pub fn vTaskDelay(ticks: u32);
    pub fn consolePutchar(ch: i32) -> i32;
}

fn console_print(msg: &str) {
    for c in msg.as_bytes() {
        unsafe{ consolePutchar(*c as i32); }
    }
}

#[no_mangle]
pub extern "C" fn appMain() -> i32 {
    console_print("Hello from Rust!\n");

    loop {
        unsafe { vTaskDelay(1000); }
    }
}
