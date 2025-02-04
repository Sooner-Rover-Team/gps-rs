//! # `soro_gps`
//!
//! Connects to the GPS and provides its data in a readable format.
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
//! ## Development
//!
//! To make changes, [install Rust](https://rustup.rs).
//!
//! ### Testing
//!
//! You'll want to run the tests before making releases. It's good to test everything concurrently, so consider using `cargo nextest` if you have it installed.

use core::net::{IpAddr, Ipv4Addr};
use std::time::Duration;

use error::{GpsConnectionError, GpsReadError};
use soro_sbp_gps::{
    variants::{Message, NavMsg},
    SbpFrame,
};
use tokio::net::UdpSocket;
use tokio_stream::StreamExt as _;
use tokio_util::{codec::BytesCodec, udp::UdpFramed};

pub mod error;

/// The best possible update time for the GPS.
///
/// This is 1/20th of a second.
pub const BEST_UPDATE_TIME: Duration = Duration::from_millis(1000 / 20);

/// A safe wrapper for the low-level GPS functions.
#[derive(Debug)]
pub struct Gps {
    /// A UDP socket to speak with the GPS directly.
    ///
    /// This allows us to get frames from the GPS.
    framed_socket: UdpFramed<BytesCodec>,
}

impl Gps {
    /// Attempts to initialize the GPS.
    ///
    /// ## Errors
    ///
    /// This constructor can fail if the GPS device is not available on the
    /// provided IP address and port.
    #[tracing::instrument]
    pub async fn new(
        gps_ip: IpAddr,
        gps_port: u16,
        socket_port: u16,
    ) -> Result<Self, GpsConnectionError> {
        // bind the socket to a local port
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, socket_port))
            .await
            .inspect_err(|e| {
                tracing::error!(
                    "Failed to bind to the given socket \
                        port (i.e. the port we talk to the GPS from). There may be another \
                        service on this port. err: {e}"
                );
            })?;

        // connect to the gps and check its file descriptor
        socket.connect((gps_ip, gps_port)).await.inspect_err(|e| tracing::warn!("Failed to connect to the given IP and port. The GPS may not be accessible. err: {e}"))?;

        // make into a UdpFramed so we can get a stream outta it
        let framed_socket = UdpFramed::new(socket, BytesCodec::new());

        Ok(Self { framed_socket })
    }

    /// Attempts to get the required GPS message.
    ///
    /// This can run forever! Run it on a background task if that won't work
    /// for your usage.
    pub async fn get(&mut self) -> Result<GpsInfo, GpsReadError> {
        // make a stream
        while let Some(udp_frame) = self.framed_socket.next().await {
            // try and grab a frame
            let bytes = match udp_frame {
                Ok((bytes, _socket_addr)) => bytes,
                Err(e) => {
                    tracing::warn!("Failed to read from socket! err: {e}");
                    continue;
                }
            };

            // try to parse whatever we got
            let (_, message) = match SbpFrame::parse(&bytes) {
                Ok(fm) => fm,
                Err(e) => {
                    tracing::warn!("Failed to parse this message! err: {e}");
                    continue;
                }
            };

            // we exclusively use this message type lol.
            if let Message::Navigation(NavMsg::MsgPosLlh(raw_coord)) = message {
                let info = GpsInfo {
                    coord: Coordinate {
                        lat: raw_coord.lat,
                        lon: raw_coord.lon,
                    },
                    height: Height(raw_coord.height),
                    tow: TimeOfWeek(raw_coord.tow),
                };

                tracing::debug!("Got all info from GPS! info: \n    {info}");
                return Ok(info);
            }
        }

        // this happens if the socket has nothing left in the stream.
        //
        // which i don't expect to happen but whatever, nice to handle it anyways
        tracing::error!("Socket has stopped providing information!");
        Err(GpsReadError::ReadFailed)
    }
}

/*
/// A safe wrapper for the low-level GPS functions.
#[derive(Debug)]
pub struct Gps {
    /// The SBP parser and the message it produces.
    parser: Option<(soro_sbp_gps::SbpFrame, soro_sbp_gps::variants::Message)>,

    /// The time we last updated the frame.
    last_updated: Option<Instant>,

    /// A UDP socket to speak with the GPS directly.
    ///
    /// This allows us to get frames from the GPS.
    framed_socket: UdpFramed<BytesCodec>,

    /// A buffer containing the most recent frame data.
    buf: [u8; soro_sbp_gps::MAX_FRAME_SIZE as usize],

    /// The timeout before giving up on a read.
    timeout: Duration,
}

impl Gps {
    /// Attempts to update our internal state with the GPS' values.
    ///
    /// note: We only actually perform updates when the GPS has had time to
    /// get a new message.
    async fn update(&mut self) -> Result<(), GpsReadError> {
        // only update if we've had enough time to get a new frame
        if let Some(last_updated) = self.last_updated {
            let elapsed = last_updated.elapsed();
            if elapsed < BEST_UPDATE_TIME {
                tracing::debug!("Asked for an update, but we don't need to do so yet.");
                return Err(GpsReadError::HaventHitUpdateTime { elapsed });
            }
        }

        // try reading from the socket
        let recv_fut = self.socket.recv(&mut self.buf);
        let _bytes_read = tokio::time::timeout(self.timeout, recv_fut)
            .await
            .map_err(|elp| {
                tracing::debug!("Hit timeout. elapsed: {elp}");
                GpsReadError::HitTimeout(self.timeout)
            })?
            .map_err(|e| {
                tracing::warn!("Read failed! err: {e}");
                GpsReadError::ReadFailed
            })
            .inspect(|bytes_read| tracing::debug!("Read {bytes_read} bytes!"))?;

        // try parsing
        let parser = SbpFrame::parse(&self.buf)?;

        // if it worked, update self.last_updated
        self.last_updated = Some(Instant::now());
        self.parser = Some(parser);

        tracing::debug!("Finished updating!");
        Ok(())
    }
}
*/

/// Information from the GPS.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct GpsInfo {
    pub coord: Coordinate,
    pub height: Height,
    // pub error_mm: ErrorInMm, // FIXME: is this still unimplemented for our gps model? try updating?
    pub tow: TimeOfWeek,
}

impl core::fmt::Display for GpsInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "GpsInfo {{ coord(lat: {}, lon: {}), height: {} m above wsg84, tow: {} ms }}",
            self.coord.lat, self.coord.lon, self.height.0, self.tow.0
        )
    }
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

/// The GPS time of week, given from the satellite. This is measured in
/// milliseconds since the start of this GNSS week.
///
/// For additional information, see
/// [the NOAA documentation](https://www.ngs.noaa.gov/CORS/Gpscal.shtml).
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct TimeOfWeek(pub u32);
