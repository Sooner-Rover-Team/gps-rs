//! A reimplementation of `gps.c` in Rust.
//!
//! But, yaknow... without the awful threading practices and likely undefined
//! behavior.
//!
//! In general, you should use the `Gps` type instead of this directly.
//!
//! This is exposed just in case someone has a use for it, avoiding an
//! impractical refactor.

use core::{error::Error, mem::ManuallyDrop, net::IpAddr};
use std::{
    ffi::c_void,
    net::{Ipv4Addr, UdpSocket},
    os::fd::AsRawFd,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread::JoinHandle,
    time::Duration,
};

use crate::bindings::{
    gps_time_node, pos_llh_node, sbp_process, sbp_register_callback, sbp_state_init, sbp_state_t,
    SBP_MSG_BASELINE_NED, SBP_MSG_DOPS, SBP_MSG_GPS_TIME, SBP_MSG_POS_LLH, SBP_MSG_VEL_NED,
};

/// A wrapper struct to let the binding be Send + Sync.
#[derive(Clone, Debug)]
pub struct SbpStateT(sbp_state_t);

unsafe impl Send for SbpStateT {}
unsafe impl Sync for SbpStateT {}

/// A thread with some data running in the background.
///
/// Collects GPS data for later use.
pub struct GpsThread {
    // /// A socket connecting to the GPS.
    // socket: Arc<UdpSocket>,
    /// when set to `true`, the thread will gracefully go down.
    stop_indicator: Arc<AtomicBool>,

    /// Stores the current info about the GPS messages.
    sbp_state: Arc<Mutex<SbpStateT>>,
}

impl GpsThread {
    /// Attempts to bring up the GPS.
    ///
    /// This does **not** start the thread. Please do so with the `start`
    /// method, and keep the thread handle around.
    #[tracing::instrument]
    pub fn new(gps_ip: IpAddr, gps_port: u16, socket_port: u16) -> Result<Self, GpsThreadError> {
        // bind the socket to a local port
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, socket_port)).inspect_err(|e| {
            tracing::error!(
                "Failed to bind to the given socket \
                port (i.e. the port we talk to the GPS from). There may be another \
                service on this port. err: {e}"
            );
        })?;

        // add a timeout to the socket.
        //
        // TODO: make this configurable? at least a thread-local?
        socket
            .set_read_timeout(Some(Duration::from_millis(200)))
            .inspect_err(|e| {
                tracing::error!("Failed to set read timeout on GPS socket. err: {e}")
            })?;

        // connect to the gps and check its file descriptor
        socket.connect((gps_ip, gps_port)).inspect_err(|e| tracing::warn!("Failed to connect to the given IP and port. The GPS may not be accessible. err: {e}"))?;
        let socket_fd = socket.as_raw_fd();
        tracing::debug!("Created socket! FD: {}", socket_fd);

        // an "sbp" message is some package of data from the gps.
        //
        // here, we set up the state used by the thead in the background.
        //
        // SAFETY: it's initialized as zero'd memory so the initialization
        // function doesn't act up.
        let mut s: sbp_state_t = unsafe { core::mem::zeroed() };

        // make the state a pointer, since all the c funcs kinda want that. lol
        let s_ptr = core::ptr::from_mut(&mut s);

        // SAFETY: we zeroed the memory, so we're not uninitialized in here.
        // in a sec, we'll also add all the callbacks into the fields.
        unsafe { sbp_state_init(s_ptr) };
        tracing::debug!("Initialized SBP state!");

        // SAFETY: uhhh well.
        // 1. the C code accepts any given type.
        // 2. when we cast the heap-allocated socket to `c_void`, we're still
        //    respecting its heap-allocated-ness
        // 3. the same type will be used in all other contexts when we're
        //    talking about this type.
        // 4. we use `ManuallyDrop` to prevent the file descriptor from being
        //    dropped when we're playing with pointers.
        let leaked_ctx = Box::leak::<'static>(Box::new(ManuallyDrop::new(socket)));
        s.io_context = core::ptr::from_mut(leaked_ctx) as *mut c_void;
        tracing::debug!("Added SBP state to C library context.");

        // debug check: ensure that we can follow the ptrs
        debug_assert_eq!(
            unsafe { core::ptr::read(s.io_context as *mut ManuallyDrop<UdpSocket>) }.as_raw_fd(),
            socket_fd,
            "file descriptors should be equal despite ptr"
        );

        // debug check: we should be able to read from it, too
        if cfg!(debug_assertions) {
            let udp_socket =
                unsafe { core::ptr::read(s.io_context as *mut ManuallyDrop<UdpSocket>) };
            let mut buf = Vec::new();

            // read from the socket until our buffer is full (or we run out of input)
            let read_bytes = udp_socket
                .recv(buf.as_mut_slice())
                .inspect_err(|e| tracing::warn!("Failed to read from socket! err: {e}"))
                .unwrap_or(0) as u32;

            tracing::debug!("Debug check success: read {read_bytes} bytes from the socket.");
        }

        // add callbacks :)
        unsafe {
            // time of week
            sbp_register_callback(
                s_ptr,
                SBP_MSG_GPS_TIME as u16,
                Some(time_callback),
                core::ptr::null_mut(),
                &raw mut gps_time_node,
            );

            // position
            sbp_register_callback(
                s_ptr,
                SBP_MSG_POS_LLH as u16,
                Some(pos_callback),
                core::ptr::null_mut(),
                &raw mut pos_llh_node,
            );

            // baseline
            sbp_register_callback(
                s_ptr,
                SBP_MSG_BASELINE_NED as u16,
                Some(baseline_callback),
                core::ptr::null_mut(),
                &raw mut pos_llh_node,
            );

            // velocity
            sbp_register_callback(
                s_ptr,
                SBP_MSG_VEL_NED as u16,
                Some(velocity_callback),
                core::ptr::null_mut(),
                &raw mut pos_llh_node,
            );

            // precision
            sbp_register_callback(
                s_ptr,
                SBP_MSG_DOPS as u16,
                Some(precision_callback),
                core::ptr::null_mut(),
                &raw mut pos_llh_node,
            );

            // imu info
            // sbp_register_callback(s_ptr, SBP_IMU, cb, context, node)
        }

        Ok(Self {
            stop_indicator: Arc::new(AtomicBool::new(false)),
            sbp_state: Arc::new(Mutex::new(SbpStateT(s))),
        })
    }
}

impl GpsThread {
    /// Starts the GPS thread yielding a thread handle.
    ///
    /// To stop the GPS thread, just call `stop` and drop the handle.
    #[tracing::instrument(skip(self))]
    pub fn start(&mut self) -> Result<JoinHandle<()>, GpsThreadError> {
        let stop_indicator = Arc::clone(&self.stop_indicator);

        // grab the state
        let state = Arc::clone(&self.sbp_state);

        // this defines an anonymous closure that's running in the background,
        // on the thread we just spawned.
        //
        // we return the join handle for users to stop the thread when ready.
        Ok(std::thread::spawn(move || {
            // move state ptr over to thread
            let state = state;
            let stop_indicator = stop_indicator;

            // we'll grab new gps data 20 times a second.
            //
            // (the GPS-RTK2 is limited to 20hz no matter what)
            //
            // the loop will stop when the stop indicator is `true`.
            while !stop_indicator.load(Ordering::Acquire) {
                tracing::info!("stop indicator: {}", stop_indicator.load(Ordering::Acquire));

                // we'll wait to access it
                let mut s = state
                    .lock()
                    .inspect_err(|e| tracing::error!("Mutex reported an error! err: {e}"))
                    .expect("lock should be available");

                // run through all the callbacks (in C)
                //
                // SAFETY: this function should be properly implemented by the
                // underlying library.
                unsafe { sbp_process(&raw mut s.0, Some(read_socket)) };

                // sleep for 1/20 seconds.
                //
                // this avoids wasting CPU cycles like the C implementation does...
                std::thread::sleep(Duration::from_millis(50));
            }

            tracing::info!("The stop indicator was given. GPS Thread will now go down!");
        }))
    }

    /// Tells the thread to stop running.
    ///
    /// Remember to join or drop the thread handle after doing this.
    pub fn stop(&mut self) {
        self.stop_indicator.store(true, Ordering::Release);
    }
}

/// An error that occurs when attempting to create or speak to the GPS thread.
#[derive(Debug, pisserror::Error)]
pub enum GpsThreadError {
    #[error("Connection failed. err: {_0}")]
    ConnectionError(#[from] std::io::Error),
}

/// A message of information received from the GPS.
///
/// The fields here may be `None`. If that's the case, the GPS hasn't yet
/// yielded another message.
pub struct GpsMessage {
    /// The time of week, in milliseconds, from the start of the week.
    ///
    /// This value stems from the satellite, not the GPS device.
    time_of_week: u32,
}

// (private) callbacks

unsafe extern "C" fn time_callback(sender_id: u16, len: u8, msg: *mut u8, context: *mut c_void) {
    todo!()
}

unsafe extern "C" fn pos_callback(sender_id: u16, len: u8, msg: *mut u8, context: *mut c_void) {
    todo!()
}

unsafe extern "C" fn baseline_callback(
    sender_id: u16,
    len: u8,
    msg: *mut u8,
    context: *mut c_void,
) {
    todo!()
}

unsafe extern "C" fn velocity_callback(
    sender_id: u16,
    len: u8,
    msg: *mut u8,
    context: *mut c_void,
) {
    todo!()
}

unsafe extern "C" fn precision_callback(
    sender_id: u16,
    len: u8,
    msg: *mut u8,
    context: *mut c_void,
) {
    todo!()
}

unsafe extern "C" fn imu_callback(sender_id: u16, len: u8, msg: *mut u8, context: *mut c_void) {
    todo!()
}

// this one function helps with reading from the socket
#[tracing::instrument]
unsafe extern "C" fn read_socket(buf: *mut u8, buf_len: u32, context: *mut c_void) -> u32 {
    // we'll cast the context from void* and read both ptrs.
    let udp_socket_ptr = context as *mut ManuallyDrop<UdpSocket>; //
                                                                  // note: since our socket is non-blocking, we have to check if we need to
                                                                  // try again. that's what this loop does - it will break pretty quickly!
    let udp_socket = &mut core::ptr::read(udp_socket_ptr);

    // debug print the socket's fd to assist in debugging ub lol
    tracing::debug!("UDP socket has FD: {}", udp_socket.as_raw_fd());

    // make the naive c buffer into a fat pointer
    let fat_buf = core::slice::from_raw_parts_mut(buf, buf_len as usize);

    // read from the socket until our buffer is full (or we run out of input)
    udp_socket
        .recv(fat_buf)
        .inspect_err(|e| tracing::warn!("Failed to read from socket! err: {e}"))
        .unwrap_or(0) as u32
}

#[cfg(test)]
mod tests {
    use super::GpsThread;
    use std::{ffi::c_void, mem::ManuallyDrop, net::UdpSocket, thread::sleep, time::Duration};

    /// simply creating a thread should work out alright.
    #[test]
    fn creation() {
        tracing_subscriber::fmt()
            .with_max_level(tracing::Level::DEBUG)
            .init();

        let mut gps = GpsThread::new("127.0.0.1".parse().unwrap(), 12345, 23456).unwrap();

        // start the thread
        let _handle = gps.start().unwrap();

        // wait a sec and join it
        std::thread::sleep(Duration::from_millis(500));
        gps.stop();
    }

    /// we should be able to use a socket from another thread without dropping
    /// the file descriptor.
    ///
    /// this kinda tests the internal logic above without having to split
    /// everything into a zillion functions. :D
    #[test]
    fn make_socket() {
        tracing_subscriber::fmt()
            .with_max_level(tracing::Level::DEBUG)
            .init();

        struct SendSyncPtr(*mut c_void);
        unsafe impl Send for SendSyncPtr {}
        unsafe impl Sync for SendSyncPtr {}

        // box a socket, leak on the heap, make a void ptr to it
        let socket = SendSyncPtr({
            let some_socket = UdpSocket::bind("127.0.0.1:9999").unwrap();
            some_socket
                .set_read_timeout(Some(Duration::from_millis(120)))
                .unwrap();
            some_socket.connect("127.0.0.1:9998").unwrap();
            let boxed_socket = Box::new(ManuallyDrop::new(some_socket));
            let leaked = Box::leak::<'static>(boxed_socket);

            // cast to ptr
            let ptr = leaked as *mut _;

            // ... and cast that to a void ptr
            ptr as *mut c_void
        });

        // spawn a friend for it to talk to
        std::thread::spawn(|| {
            let sender_socket = UdpSocket::bind("127.0.0.1:9998").unwrap();
            let mut k = 0_u8;

            loop {
                sender_socket.send_to(&[k], "127.0.0.1:9999").unwrap();
                k = k.wrapping_add(1);
            }
        });

        // and finally, stick our pointer in a thread and try and speak to it
        let reader = std::thread::spawn(move || {
            let wrapped_ptr = socket;
            let ptr = wrapped_ptr.0 as *mut ManuallyDrop<UdpSocket>;
            let mut buf = vec![];

            let socket = &mut unsafe { ptr.read() };
            let _recvd = socket.recv(&mut buf).unwrap();
            let _recvd = socket.recv(&mut buf).unwrap();
            tracing::debug!("did two reads :D");

            for i in 0..=50 {
                let socket = unsafe { ptr.read() };
                let _recvd = socket.recv(&mut buf).unwrap();
                tracing::debug!("on iteration {i}, read: `{buf:x?}`");

                buf.clear();
                sleep(Duration::from_millis(20));
                tracing::debug!("finished a loop. i: {i}");
            }
        });

        sleep(Duration::from_secs(2));
        reader.join().unwrap();
    }
}
