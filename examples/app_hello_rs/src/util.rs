pub fn console_print(msg: &str) {
    for &b in msg.as_bytes() {
        unsafe {
            crate::ffi::consolePutchar(b as i32);
        }
    }
}