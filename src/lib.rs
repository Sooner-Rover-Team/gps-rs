//! # `gps-rs`
//!
//! Bindings to the Swift GPS library ([`gps/`](https://github.com/Sooner-Rover-Team/gps)).
//!
//! Currently, these are unsafe bindings for use in Rust only. In the future, they should be extended to expose safe bindings with lots of documentation and respect for the (kinda undocumented) safety constraints of the C code. Previous testing shows that violating these unspoken invariants can result in all kinds of weird behavior!

#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

// pub use self::{
//     get_error, get_error, get_height, get_latitude, get_longitude, get_time,
//     gps_finish, gps_init, gps_thread,
// };
