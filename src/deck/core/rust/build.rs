extern crate bindgen;

use std::env;
use std::path::PathBuf;

fn main() {
    // Tell cargo to invalidate the built crate whenever the wrapper changes
    println!("cargo:rerun-if-changed=wrapper.h");

    let target = env::var("TARGET").unwrap();

    // The bindgen::Builder is the main entry point
    // to bindgen, and lets you build up options for
    // the resulting bindings.
    let bindings = bindgen::Builder::default()
        // The input header we would like to generate
        // bindings for.
        .use_core()
        .ctypes_prefix("cty")
        .header("wrapper.h")
        .clang_args(&["-target", &target])
        // Set the include paths
        .clang_arg("-I../../interface")
        .clang_arg("-I../../../modules/interface")
        .clang_arg("-I../../../hal/interface")
        .clang_arg("-I../../../drivers/interface")
        .clang_arg("-I../../../utils/interface/")
        .clang_arg("-I../../../utils/interface/lighthouse")
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("deck_bindings.rs"))
        .expect("Couldn't write bindings!");
}
