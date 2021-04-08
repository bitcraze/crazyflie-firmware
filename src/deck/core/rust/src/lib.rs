#![no_std]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

include!(concat!(env!("OUT_DIR"), "/deck_bindings.rs"));

//
// Below is a redefinition of the DeckDriver in order to mark it as thread safe
// so that we can use a pointer to it as a global static at a certain link
// section.
//
// We also need to set the RequiredEstimator field to be u16, since the
// firmware has short enums.
//
#[repr(C)]
#[derive(Debug, Clone)]
pub struct ConstDeckMemDef(pub *const deckMemDef_s);
unsafe impl Sync for ConstDeckMemDef {}

#[repr(C)]
#[derive(Debug, Clone)]
pub struct DriverName(pub *const cty::c_char);
unsafe impl Sync for DriverName {}

#[repr(C)]
#[derive(Debug, Clone)]
pub struct deck_driver_unsafe {
    pub vid: u8,
    pub pid: u8,
    pub name: DriverName,
    pub usedPeriph: u32,
    pub usedGpio: u32,
    pub requiredEstimator: u16,
    pub requiredLowInterferenceRadioMode: bool,
    pub memoryDef: ConstDeckMemDef,
    pub init: unsafe extern "C" fn(arg1: *mut deckInfo_s),
    pub test: unsafe extern "C" fn() -> bool,
}

pub type RustDeckDriver = deck_driver_unsafe;

#[repr(C)]
pub struct ConstDeckDriver(pub *const RustDeckDriver);
unsafe impl Sync for ConstDeckDriver {}

//
// This macro places a pointer to a RustDeckDriver at a certain link section
// and marks it as used so that is not cleaned away. The first argument is the
// struct to point at and the other is an ident that needs to be globally
// unique.
//
#[macro_export]
macro_rules! deck_driver {
    ($name:ident, $driver:ident) => {
        #[no_mangle]
        #[link_section = concat!(".deckDriver.", stringify!($name))]
        #[used]
        static $driver: ConstDeckDriver = ConstDeckDriver(&$name as *const _);
    };
}

#[panic_handler]
fn panic(_panic: &core::panic::PanicInfo<'_>) -> ! {
    loop {}
}

#[macro_export]
macro_rules! console_print {
    ($msg:expr) => {
        unsafe {
            eprintf(Some(consolePutchar), $msg);
        }
    };
}
