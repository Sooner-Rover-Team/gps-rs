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
    gps_time_node, msg_baseline_ned_t, msg_dops_t, msg_gps_time_t, msg_imu_raw_t, msg_pos_llh_t,
    msg_vel_ned_t, pos_llh_node, sbp_process, sbp_register_callback, sbp_state_init, sbp_state_t,
    SBP_MSG_BASELINE_NED, SBP_MSG_DOPS, SBP_MSG_GPS_TIME, SBP_MSG_IMU_RAW, SBP_MSG_POS_LLH,
    SBP_MSG_VEL_NED,
};

/// A wrapper struct to let the binding be Send + Sync.
#[derive(Clone, Debug)]
pub struct SbpStateT(*mut sbp_state_t);

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

    /// The most recent data available from the GPS.
    ///
    /// This is a heap-allocated pointer so we can box + drop it at the end of
    /// our scope, handling the otherwise-leaked memory.
    data_ptr: *mut Mutex<GpsData>,

    /// A heap-allocated pointer for the state of the GPS. It's used internally
    /// by the C library - we don't ever touch this except to feed it to the
    /// thread we run it all on.
    ///
    /// We'll re-box it when we're dropped to 'unleak' the memory.
    ///
    /// See the `data_ptr` for addition information.
    state_ptr: SbpStateT,
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
        let mut state: sbp_state_t = unsafe { core::mem::zeroed() };

        // now, we'll leak that onto the heap, then rebox it when we're dropped
        let boxed_state = Box::new(state);

        // make the state a pointer, since all the c funcs kinda want that. lol
        let state_ptr = core::ptr::from_mut(Box::leak::<'static>(boxed_state));

        // SAFETY: we zeroed the memory, so we're not uninitialized in here.
        // in a sec, we'll also add all the callbacks into the fields.
        unsafe { sbp_state_init(state_ptr) };
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
        state.io_context = core::ptr::from_mut(leaked_ctx) as *mut c_void;

        // debug check: ensure that we can follow the ptrs
        debug_assert_eq!(
            unsafe { core::ptr::read(state.io_context as *mut ManuallyDrop<UdpSocket>) }
                .as_raw_fd(),
            socket_fd,
            "file descriptors should be equal despite ptr"
        );

        // debug check: we should be able to read from it, too
        if cfg!(debug_assertions) {
            let udp_socket =
                unsafe { core::ptr::read(state.io_context as *mut ManuallyDrop<UdpSocket>) };
            let mut buf = Vec::new();

            // read from the socket until our buffer is full (or we run out of input)
            let read_bytes = udp_socket
                .recv(buf.as_mut_slice())
                .inspect_err(|e| tracing::warn!("Failed to read from socket! err: {e}"))
                .unwrap_or(0) as u32;

            tracing::debug!("Debug check success: read {read_bytes} bytes from the socket.");
        }

        // make an empty data struct to read messages into.
        //
        // we'll leak it into a pointer, then deallocate that when we're
        // dropped.
        //
        // this prevents the callbacks from referencing a dangling pointer,
        // but also lets us be more certain about the `*mut` type we cast to.
        let data = Box::new(Mutex::new(GpsData::default()));
        let data_ptr = core::ptr::from_mut(Box::leak::<'static>(data));

        // add callbacks :)
        unsafe {
            // time of week
            sbp_register_callback(
                state_ptr,
                SBP_MSG_GPS_TIME as u16,
                Some(callbacks::time_callback),
                data_ptr as *mut c_void,
                &raw mut gps_time_node,
            );

            // position
            sbp_register_callback(
                state_ptr,
                SBP_MSG_POS_LLH as u16,
                Some(callbacks::pos_callback),
                data_ptr as *mut c_void,
                &raw mut pos_llh_node,
            );

            // baseline
            sbp_register_callback(
                state_ptr,
                SBP_MSG_BASELINE_NED as u16,
                Some(callbacks::baseline_callback),
                data_ptr as *mut c_void,
                &raw mut pos_llh_node,
            );

            // velocity
            sbp_register_callback(
                state_ptr,
                SBP_MSG_VEL_NED as u16,
                Some(callbacks::velocity_callback),
                data_ptr as *mut c_void,
                &raw mut pos_llh_node,
            );

            // precision
            sbp_register_callback(
                state_ptr,
                SBP_MSG_DOPS as u16,
                Some(callbacks::precision_callback),
                data_ptr as *mut c_void,
                &raw mut pos_llh_node,
            );

            // imu info
            sbp_register_callback(
                state_ptr,
                SBP_MSG_IMU_RAW as u16,
                Some(callbacks::imu_callback),
                data_ptr as *mut c_void,
                &raw mut pos_llh_node,
            );
        }

        Ok(Self {
            stop_indicator: Arc::new(AtomicBool::new(false)),
            data_ptr,
            state_ptr: SbpStateT(state_ptr),
        })
    }

    /// Starts the GPS thread yielding a thread handle.
    ///
    /// To stop the GPS thread, just call `stop` and drop the handle.
    #[tracing::instrument(skip(self))]
    pub fn start(&mut self) -> Result<JoinHandle<()>, GpsThreadError> {
        let stop_indicator = Arc::clone(&self.stop_indicator);

        // grab the state
        //
        // SAFETY: the C library is responsible for all future mutation.
        // we do not modify its contents manually whatsoever past its initial
        // construction.
        let state = self.state_ptr.clone();

        // this defines an anonymous function (called a closure) that's running
        // in the background. that's the thread we just spawned!
        //
        // we return the join handle for users to stop the thread when ready.
        Ok(std::thread::spawn(move || {
            // move stuff to thread.
            //
            // this is required to prevent compiler errors - we're literally
            // moving these values to the thread.
            let state = state;
            let stop_indicator = stop_indicator;

            // grab the ptr for the c lib.
            let state_ptr = state.0;

            // we'll grab new gps data 20 times a second.
            //
            // (the GPS-RTK2 is limited to 20hz no matter what)
            //
            // the loop will stop when the stop indicator is `true`.
            while !stop_indicator.load(Ordering::Acquire) {
                // run through all the callbacks (in C)
                //
                // SAFETY: this function should be properly implemented by the
                // underlying library.
                unsafe { sbp_process(state_ptr, Some(read_socket)) };

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
    #[tracing::instrument(skip(self))]
    pub fn stop(&mut self) {
        self.stop_indicator.store(true, Ordering::Release);
        tracing::debug!("Set stop flag for thread.");
    }
}

impl Drop for GpsThread {
    fn drop(&mut self) {
        // deallocate the 'leaked' memory by putting it back into a box,
        // then dropping that box.
        //
        // SAFETY: this fn is only ever called once, so not a double-free.
        // also, since Box allocated this memory, it's also layout-compatible.
        let boxed_data = unsafe { Box::from_raw(self.data_ptr) };
        drop(boxed_data);

        // do the same for the state.
        //
        // SAFETY: see above.
        let boxed_state = unsafe { Box::from_raw(self.state_ptr.0) };
        drop(boxed_state);
    }
}

/// An error that occurs when attempting to create or speak to the GPS thread.
#[derive(Debug, pisserror::Error)]
pub enum GpsThreadError {
    #[error("Connection failed. err: {_0}")]
    ConnectionError(#[from] std::io::Error),
}

/// A message of information received from the GPS.
#[derive(Clone, Debug, Default)]
pub struct GpsData {
    pub pos: Option<msg_pos_llh_t>,
    pub baseline: Option<msg_baseline_ned_t>,
    pub velocity: Option<msg_vel_ned_t>,
    pub precision: Option<msg_dops_t>,
    pub time_of_week: Option<msg_gps_time_t>,
    pub imu_raw: Option<msg_imu_raw_t>,
}

// this one function helps with reading from the socket
/// void pointers are basically veeeery scuffed generics.
///
/// For more info on void pointers, see
/// [this Wikipedia article](https://en.wikipedia.org/wiki/Pointer_(computer_programming)#C_and_C++).
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

/// Callbacks
///
/// Each of these follows the same structure.
///
/// We assume the C library didn't feed us a junk pointer, grab the contents of
/// the Mutex, and modify the appropriate field (if we got a lock).
mod callbacks {
    use std::ffi::c_void;

    #[tracing::instrument]
    pub(super) unsafe extern "C" fn pos_callback(
        _sender_id: u16,
        _len: u8,
        msg: *mut u8,
        context: *mut c_void,
    ) {
        todo!();
    }

    #[tracing::instrument]
    pub(super) unsafe extern "C" fn baseline_callback(
        _sender_id: u16,
        _len: u8,
        msg: *mut u8,
        context: *mut c_void,
    ) {
        todo!();
    }

    #[tracing::instrument]
    pub(super) unsafe extern "C" fn velocity_callback(
        _sender_id: u16,
        _len: u8,
        msg: *mut u8,
        context: *mut c_void,
    ) {
        todo!();
    }

    #[tracing::instrument]
    pub(super) unsafe extern "C" fn precision_callback(
        _sender_id: u16,
        _len: u8,
        msg: *mut u8,
        context: *mut c_void,
    ) {
        todo!();
    }

    #[tracing::instrument]
    pub(super) unsafe extern "C" fn time_callback(
        _sender_id: u16,
        _len: u8,
        msg: *mut u8,
        context: *mut c_void,
    ) {
        todo!();
    }

    #[tracing::instrument]
    pub(super) unsafe extern "C" fn imu_callback(
        _sender_id: u16,
        _len: u8,
        msg: *mut u8,
        context: *mut c_void,
    ) {
        todo!();
    }
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
