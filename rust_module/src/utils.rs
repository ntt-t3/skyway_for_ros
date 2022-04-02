use std::ffi::{c_void, CString};
use std::net::TcpListener;
use std::os::raw::c_char;
use std::thread::JoinHandle;

use shaku::{Component, Interface};

use crate::ffi::TopicParameters;
use crate::CallbackFunctions;

#[cfg(test)]
use mockall::automock;

#[no_mangle]
pub extern "C" fn release_string(message: *mut c_char) {
    unsafe {
        let _ = CString::from_raw(message);
    }
}

#[no_mangle]
pub extern "C" fn join_handler(handler: *mut c_void) {
    let handle = unsafe { Box::from_raw(handler as *mut JoinHandle<()>) };
    let _ = handle.join();
}

pub(crate) fn available_port() -> std::io::Result<u16> {
    match TcpListener::bind("0.0.0.0:0") {
        Ok(listener) => Ok(listener.local_addr().unwrap().port()),
        Err(e) => Err(e),
    }
}

#[cfg_attr(test, automock)]
pub(crate) trait CallbackCaller: Interface {
    fn create_peer_callback(&self, peer_id: &str, token: &str);
    fn peer_deleted_callback(&self);
    fn data_callback(&self, param: TopicParameters);
    fn data_connection_deleted_callback(&self, data_connection_id: &str);
}

#[derive(Component)]
#[shaku(interface = CallbackCaller)]
pub(crate) struct CallbackCallerImpl {}

impl CallbackCaller for CallbackCallerImpl {
    fn create_peer_callback(&self, peer_id: &str, token: &str) {
        let callback = CallbackFunctions::global();
        callback.create_peer_callback(peer_id, token);
    }

    fn peer_deleted_callback(&self) {
        let callback = CallbackFunctions::global();
        callback.peer_deleted_callback();
    }

    fn data_callback(&self, param: TopicParameters) {
        let callback = CallbackFunctions::global();
        callback.data_callback(param);
    }

    fn data_connection_deleted_callback(&self, data_connection_id: &str) {
        let callback = CallbackFunctions::global();
        callback.data_connection_deleted_callback(data_connection_id);
    }
}
