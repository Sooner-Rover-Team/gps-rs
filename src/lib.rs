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
//! ```no_run
//! # use core::net::IpAddr;
//! use soro_gps::Gps;
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
//! ```python
//! from soro_gps import Gps
//!
//! # make a gps from an ip str + port int
//! gps: Gps = Gps("192.168.1.222", 55556)
//!
//! # grab values from methods
//! print(f"coord: {gps.coord()}")
//! print(f"height: {gps.height()}")
//! print(f"error: {gps.error()}")
//! ```
//!
//! ## Development
//!
//! To make small changes on the Rust side of things, you won't need any Python tooling until you make a release. Just [install Rust](https://rustup.rs).
//!
//! For anything past that (especially Python development), you'll want to get `maturin`. To do so, [download `cargo-binstall`](https://github.com/cargo-bins/cargo-binstall?tab=readme-ov-file#quickly), then run `cargo binstall maturin`.
//!
//! Afterwards, you can build the Python wheel with `maturin build --release --out dist --interpreter python3.13`. That'll result in a `.whl` ("wheel") file in the `dist/` directory. Distribute it on PyPi by running `maturin publish`. Alternatively, you can push it into your virtual environment with `pip install soro_gps --find-links dist --force-reinstall`.
//!
//! ### Testing
//!
//! You'll want to run the tests before

#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use core::error::Error;
use std::net::IpAddr;

use thread::{GpsThread, GpsThreadError};

pub mod thread;

/// A safe wrapper for the low-level GPS functions.
#[derive(Debug)]
#[cfg_attr(feature = "python", pyo3::pyclass)]
pub struct Gps {
    /// The internal `gps.c` reimplementation.
    ///
    /// It interfaces with the lower-level parts of the C library for us.
    thread: GpsThread,
}

impl Gps {
    /// Attempts to initialize the GPS.
    ///
    /// ## Errors
    ///
    /// This constructor can fail if the given IP and port are not able to be
    /// converted to a CString, but this should generally succeed.
    #[tracing::instrument]
    pub fn new(gps_ip: IpAddr, gps_port: u16, socket_port: u16) -> Result<Self, GpsError> {
        let thread = GpsThread::new(gps_ip, gps_port, socket_port)
            .inspect_err(|e| tracing::warn!("Failed to set up GPS thread. err: {e}"))?;

        Ok(Self { thread })
    }

    /// Finds the coordinate as determined by the GPS.
    #[tracing::instrument(skip(self))]
    pub fn coord(&self) -> Option<Coordinate> {
        let pos_data = self.thread.data()?.pos?;

        Some(Coordinate {
            lat: pos_data.lat,
            lon: pos_data.lon,
        })
    }

    /// Finds the 'height', which is an arbitrary measure with WSG84.
    ///
    /// See the [`Height`] for additional information.
    #[tracing::instrument(skip(self))]
    pub fn height(&self) -> Option<Height> {
        let pos_data = self.thread.data()?.pos?;
        Some(Height(pos_data.height))
    }

    /// Gets the known 'error' in millimeters from the GPS.
    #[tracing::instrument(skip(self))]
    pub fn error(&self) -> Option<ErrorInMm> {
        let pos_data = self.thread.data()?.pos?;

        // the following uses two millimeter accuracy values averaged.
        //
        // we convert them to f64 before addition to prevent overflows, though
        // this miiiight decrease accuracy. hoping `f64` is good enough.
        //
        // FIXME: this is ripped straight from `gps.c`, but they didn't make
        // great decisions in there. consider making this error a 'better' type
        // or something. im doubtful that averaging the two directions makes
        // for a good result. lol
        let error = (pos_data.h_accuracy as f64 + pos_data.v_accuracy as f64) / 2.0;

        Some(ErrorInMm(error))
    }

    /// From the satellite, grabs the number of milliseconds since the start of
    /// this week. Meaning the "time of week".
    pub fn time_of_week(&self) -> Option<TimeOfWeek> {
        let pos_data = self.thread.data()?.pos?;
        Some(TimeOfWeek(pos_data.tow))
    }
}

// when python is turned on, we'll create an exception
#[cfg(feature = "python")]
pyo3::create_exception!(error, GpsException, pyo3::exceptions::PyException);

#[cfg(feature = "python")]
#[cfg_attr(feature = "python", pyo3::pymethods)]
impl Gps {
    #[new]
    #[tracing::instrument]
    pub fn py_new(gps_ip: String, gps_port: u16, socket_port: u16) -> pyo3::PyResult<Self> {
        let gps_ip = gps_ip.parse()?;

        Self::new(gps_ip, gps_port, socket_port)
            .inspect_err(|e| tracing::warn!("Failed to create GPS! err: {e}"))
            .map_err(|e| GpsException::new_err(e.to_string()))
    }

    /// Finds the coordinate as determined by the GPS.
    #[pyo3(name = "coord")]
    #[tracing::instrument(skip(self))]
    pub fn py_coord(&self) -> Option<Coordinate> {
        self.coord()
    }

    /// Finds the 'height', which is an arbitrary measure with WSG84.
    ///
    /// See the [`Height`] for additional information.
    #[pyo3(name = "height")]
    #[tracing::instrument(skip(self))]
    pub fn py_height(&self) -> Option<Height> {
        self.height()
    }

    /// Gets the known 'error' in millimeters from the GPS.
    #[pyo3(name = "error")]
    #[tracing::instrument(skip(self))]
    pub fn py_error(&self) -> Option<ErrorInMm> {
        self.error()
    }

    /// Gets the known 'error' in millimeters from the GPS.
    #[pyo3(name = "time_of_week")]
    #[tracing::instrument(skip(self))]
    pub fn py_time_of_week(&self) -> Option<TimeOfWeek> {
        self.time_of_week()
    }
}

// SAFETY: we don't actually use the internal ptrs until we drop the class.
unsafe impl Send for Gps {}

// SAFETY: see above!
unsafe impl Sync for Gps {}

impl Drop for Gps {
    #[tracing::instrument(skip(self))]
    fn drop(&mut self) {
        tracing::debug!("Calling `gps_finish`...");

        // SAFETY: we do not drop the strings we gave it, as doing so can cause
        // a double-free. and the library already has one of those somewhere!
        unsafe { bindings::gps_finish() };
    }
}

/// An error that can occur when using the GPS.
#[derive(Debug, pisserror::Error)]
pub enum GpsError {
    #[error("Error occurred while constructing the GPS thread. err: {_0}")]
    GpsThreadError(#[from] GpsThreadError),
}

/// Some coordinate returned from the GPS.
///
/// The returned values are in degrees.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "python", pyo3::pyclass)]
#[cfg_attr(feature = "python", pyo3(eq))]
pub struct Coordinate {
    pub lat: f64,
    pub lon: f64,
}

#[cfg_attr(feature = "python", pyo3::pymethods)]
impl Coordinate {
    pub fn __str__(&self) -> String {
        format!("{}° lat, {}° lon", self.lat, self.lon)
    }
}

/// Height, with the unit being "meters above a WGS84 ellipsoid".
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "python", pyo3::pyclass)]
#[cfg_attr(feature = "python", pyo3(eq))]
pub struct Height(pub f64);

#[cfg_attr(feature = "python", pyo3::pymethods)]
impl Height {
    pub fn __str__(&self) -> String {
        format!("{:.2}m", self.0)
    }
}

/// Error in millimeters. Inside the `gps` library, this is calculated as the
/// average of the vertical and horizontal 'error'.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "python", pyo3::pyclass)]
#[cfg_attr(feature = "python", pyo3(eq))]
pub struct ErrorInMm(pub f64);

#[cfg_attr(feature = "python", pyo3::pymethods)]
impl ErrorInMm {
    pub fn __str__(&self) -> String {
        format!("{:.2}mm", self.0)
    }
}

/// The GPS time of week, given from the satellite. This is measured in
/// milliseconds since the start of this GNSS week.
///
/// For additional information, see
/// [the NOAA documentation](https://www.ngs.noaa.gov/CORS/Gpscal.shtml).
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "python", pyo3::pyclass)]
#[cfg_attr(feature = "python", pyo3(eq))]
pub struct TimeOfWeek(pub u32);

// export the module to python!
#[cfg_attr(feature = "python", pyo3::pymodule)]
#[cfg(feature = "python")]
fn soro_gps(m: &pyo3::Bound<'_, pyo3::types::PyModule>) -> pyo3::PyResult<()> {
    pyo3::types::PyModuleMethods::add_class::<Gps>(m)?;
    pyo3::types::PyModuleMethods::add_class::<Coordinate>(m)?;
    pyo3::types::PyModuleMethods::add_class::<Height>(m)?;
    pyo3::types::PyModuleMethods::add_class::<ErrorInMm>(m)?;
    pyo3::types::PyModuleMethods::add_class::<TimeOfWeek>(m)?;
    pyo3::types::PyModuleMethods::add(m, "GpsException", m.py().get_type::<GpsException>())?;
    Ok(())
}
