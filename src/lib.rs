//! # `gps-rs`
//!
//! Bindings to the Swift GPS library ([`gps/`](https://github.com/Sooner-Rover-Team/gps)).
//!
//! This crate exposes the `bindings` module to access the C functions and statics directly.
//!
//! However, intended usage is through the `Gps` struct, which provides a safe wrapper for the unsafe C items.
//!
//! This type exposes safe bindings with respect to the (kinda undocumented) safety constraints of the C code. Previous testing shows that violating these unspoken invariants can result in all kinds of weird behavior!
//!
//! ## Usage (Rust)
//!
//! In short, you can use the various methods on `Gps` to interact with the Rover's GPS system.
//!
//! ```
//! # use core::net::IpAddr;
//! use gps_rs::Gps;
//!
//! // we can make a GPS from a given IP address and port.
//! //
//! // on the Rover, this is from the router (hopefully a static IP) and a port
//! let swift_ip: IpAddr = "192.168.1.222".parse().unwrap();
//! let swift_port: u16 = 55556;
//!
//! // here, we make the GPS! this will automatically initialize it.
//! let gps: Gps = Gps::new(swift_ip, swift_port).unwrap();
//!
//! // now, you can use its methods:
//! println!("coord: {:?}", gps.coord());
//! println!("height: {:?}", gps.height());
//! println!("error in mm: {:?}", gps.error());
//!
//! // dropping it (when it falls from scope) automatically runs the required cleanup.
//! ```
//!
//! ## Usage (Python)
//!
//! Unimplemented. This should use the `pyo3` feature to build, and I'll plan to upload it to PyPi soon.

#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use core::error::Error;
use std::{ffi::CString, net::IpAddr};

pub mod bindings {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

/// A safe wrapper for the low-level GPS functions.
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Gps {
    /// A pointer to a boxed character array on the heap.
    ///
    /// This will automatically be deallocated when we're dropped (on the Rust
    /// side of things).
    ip_ptr: *mut i8,

    /// Same as above, but for the port.
    port_ptr: *mut i8,
}

impl Gps {
    /// Attempts to initialize the GPS.
    ///
    /// ## Errors
    ///
    /// This constructor can fail if the given IP and port are not able to be
    /// converted to a CString, but this should generally succeed.
    pub fn new(ip: IpAddr, port: u16) -> Result<Self, GpsError> {
        // FIXME: we currently don't have a way to check if the device is
        // already talking to another device! the `PTHREAD_MUTEX` may provide
        // a way around this.

        // make our c string ptrs
        let ip = CString::new(ip.to_string())
            .inspect_err(|e| {
                tracing::error!(
                    "Failed to create a C-style string from the given IP address input. err: {e}"
                )
            })?
            .into_raw();
        let port = CString::new(port.to_string())
            .inspect_err(|e| {
                tracing::error!("Failed to create a C-style string from the given port. err: {e}")
            })?
            .into_raw();

        // SAFETY: we will not deallocate the given ip + port until we also
        // stop the GPS thread. (see `Drop::drop` impl)
        unsafe {
            bindings::gps_init(ip, port);
        }

        Ok(Self {
            ip_ptr: ip,
            port_ptr: port,
        })
    }

    /// Finds the coordinate as determined by the GPS.
    pub fn coord(&self) -> Coordinate {
        // SAFETY: the GPS thread has been initialized.
        let (lat, lon) = (unsafe { bindings::get_latitude() }, unsafe {
            bindings::get_longitude()
        });

        Coordinate { lat, lon }
    }

    /// Finds the 'height', which is an arbitrary measure with WSG84.
    ///
    /// See the [`Height`] for additional information.
    pub fn height(&self) -> Height {
        // SAFETY: the GPS thread is initialized.
        let height = unsafe { bindings::get_height() };

        Height(height)
    }

    /// Gets the known 'error' in millimeters from the GPS.
    pub fn error(&self) -> ErrorInMm {
        // SAFETY: GPS thread is initialized.
        let error = unsafe { bindings::get_error() };

        ErrorInMm(error)
    }
}

impl Drop for Gps {
    #[tracing::instrument(skip(self))]
    fn drop(&mut self) {
        tracing::debug!("Calling `gps_finish`...");
        unsafe { bindings::gps_finish() };

        drop(unsafe { CString::from_raw(self.ip_ptr) });
        drop(unsafe { CString::from_raw(self.port_ptr) });
    }
}

/// An error that can occur when using the GPS.
#[derive(Clone, Debug, pisserror::Error)]
pub enum GpsError {
    #[error("Couldn't create the C-style strings required by the `gps` library. err: {_0}")]
    CStrCreationFailed(#[from] std::ffi::NulError),
}

/// Some coordinate returned from the GPS.
///
/// The returned values are in degrees.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Coordinate {
    pub lat: f64,
    pub lon: f64,
}

/// Height, with the unit being "meters above a WGS84 ellipsoid".
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Height(pub f64);

/// Error in millimeters. Inside the `gps` library, this is calculated as the
/// average of the vertical and horizontal 'error'.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct ErrorInMm(pub f64);

/*

notice: the following was removed since the library returns a `u32` as an
`f64`/`double`.

i can't tell if that implicit 'cast' will mess with the bit repr, or if it's a
true conversion into a float.

either way, it's pretty annoying, because milliseconds are pretty specific,
and `u32` otherwise works perfectly...

impl Gps {
    pub fn time_of_week(&self) -> TimeOfWeek {
        // SAFETY: GPS thread is initialized.
        let tow = unsafe { bindings::get_time() };

        // SAFETY: for some reason, the `u32` is casted to an `f64`/double in the
        // library.
        //
        // i cast it back into a `u64` here...
        let tow = unsafe { core::mem::transmute(tow) };

        // and here, i push it back into a `u32`
        let tow = tow as u32;

        TimeOfWeek(tow)
    }
}

/// The GPS time of week, given from the satellite. This is measured in
/// milliseconds since the start of this GNSS week.
///
/// For additional information, see
/// [the NOAA documentation](https://www.ngs.noaa.gov/CORS/Gpscal.shtml).
pub struct TimeOfWeek(pub u32);

*/
