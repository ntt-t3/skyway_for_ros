use std::net::TcpListener;

use shaku::{Component, Interface};

use crate::ffi::rust_to_c_bridge::c_functions_wrapper::*;

#[cfg(test)]
use mockall::automock;

#[allow(dead_code)]
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
    fn data_callback(&self, param: &str) -> PluginLoadResult;
    fn data_connection_deleted_callback(&self, data_connection_id: &str);
}

#[derive(Component)]
#[shaku(interface = CallbackCaller)]
pub(crate) struct CallbackCallerImpl {}

impl CallbackCaller for CallbackCallerImpl {
    fn create_peer_callback(&self, peer_id: &str, token: &str) {
        let callback = CallbackFunctionsHolder::global();
        callback.create_peer_callback(peer_id, token);
    }

    fn peer_deleted_callback(&self) {
        let callback = CallbackFunctionsHolder::global();
        callback.peer_deleted_callback();
    }

    fn data_callback(&self, param: &str) -> PluginLoadResult {
        let callback = CallbackFunctionsHolder::global();
        callback.data_callback(param)
    }

    fn data_connection_deleted_callback(&self, data_connection_id: &str) {
        let callback = CallbackFunctionsHolder::global();
        callback.data_connection_deleted_callback(data_connection_id);
    }
}
