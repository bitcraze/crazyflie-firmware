#![no_std]
#![feature(extended_key_value_attributes)]

use rust_drivers::*;

#[no_mangle]
pub extern "C" fn rustExampleInit(_info: *mut DeckInfo) {
    console_print!(b"Hello from rustExampleDriver init\n\0".as_ptr() as *const cty::c_char);
}

#[no_mangle]
pub extern "C" fn rustExampleTest() -> bool {
    console_print!(b"Hello from rustExampleDriver test\n\0".as_ptr() as *const cty::c_char);
    true
}

#[no_mangle]
static rust_example: RustDeckDriver = RustDeckDriver {
    name: DriverName(b"rustExampleDriver\0".as_ptr() as *const cty::c_char),
    init: rustExampleInit,
    test: rustExampleTest,
    usedPeriph: 0,
    usedGpio: 0,
    requiredEstimator: 0,
    requiredLowInterferenceRadioMode: false,
    vid: 0xDE,
    pid: 0xAD,
    memoryDef: ConstDeckMemDef(core::ptr::null() as *const DeckMemDef_t),
};

deck_driver!(rust_example, RustExampleDriver);
