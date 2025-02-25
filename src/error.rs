use core::error::Error;
use std::time::Duration;

/// An error that can occur when connecting to the GPS.
#[derive(Debug, pisserror::Error)]
pub enum GpsConnectionError {
    #[error("Failed to bind to the provided port. port: {port}, err: {err}")]
    BindError { port: u16, err: std::io::Error },

    #[error("Couldn't connect to the provided address and port. err: {_0}")]
    ConnectionError(#[from] std::io::Error),
}

/// An error that may occur when reading from the GPS.
#[derive(Debug, pisserror::Error)]
pub enum GpsReadError {
    #[error("GPS can't update that fast. elapsed: {} ms", elapsed.as_millis())]
    HaventHitUpdateTime { elapsed: Duration },

    #[error("Failed to read before hitting the timeout. timeout: {} ms", _0.as_millis())]
    HitTimeout(Duration),

    #[error("Failed to read from the socket.")]
    ReadFailed,

    #[error("Parsing failed.")]
    ParseFailed(#[from] sbp::Error),
}
