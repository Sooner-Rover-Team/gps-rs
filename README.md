<!-- cargo-rdme start -->

# `gps-rs`

Bindings to the Swift GPS library ([`gps/`](https://github.com/Sooner-Rover-Team/gps)).

This crate exposes the `bindings` module to access the C functions and statics directly.

However, intended usage is through the `Gps` struct, which provides a safe wrapper for the unsafe C items.

This type exposes safe bindings with respect to the (kinda undocumented) safety constraints of the C code. Previous testing shows that violating these unspoken invariants can result in all kinds of weird behavior!

## Usage (Rust)

In short, you can use the various methods on `Gps` to interact with the Rover's GPS system.

```rust
use gps_rs::Gps;

// we can make a GPS from a given IP address and port.
//
// on the Rover, this is from the router (hopefully a static IP) and a port
let swift_ip: IpAddr = "192.168.1.222".parse().unwrap();
let swift_port: u16 = 55556;

// here, we make the GPS! this will automatically initialize it.
let gps: Gps = Gps::new(swift_ip, swift_port).unwrap();

// now, you can use its methods:
println!("coord: {:?}", gps.coord());
println!("height: {:?}", gps.height());
println!("error in mm: {:?}", gps.error());

// dropping it (when it falls from scope) automatically runs the required cleanup.
```

## Usage (Python)

Unimplemented. This should use the `pyo3` feature to build, and I'll plan to upload it to PyPi soon.

<!-- cargo-rdme end -->
