use std::env;
use std::path::PathBuf;

fn main() {
    // make sure the gps library exists...
    let _gps_lib_path = PathBuf::from("gps/").canonicalize().expect(
        "Failed to find the `gps` library! Please make sure you cloned \
    using `git clone <github link> --recurse-submodules`.",
    );

    // compile the `gps` lib c files, which imports a bunch of important stuff
    compile();

    // tell cargo to statically link them into our crate.
    println!("cargo:rustc-link-lib=gps");

    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path: PathBuf = PathBuf::from(env::var("OUT_DIR").unwrap());

    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}

/// we need to compile the `gps` library before we can use it!
fn compile() {
    // we'll just compile these c files :)
    cc::Build::new()
        .static_flag(true)
        .files(["gps/edc.c", "gps/sbp.c", "gps/gps.c"])
        .compile("gps");
}
