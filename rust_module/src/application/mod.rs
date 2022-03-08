mod dto;
mod usecase;

use std::ffi::{CStr, CString};
use std::os::raw::c_char;

use crate::application::dto::Dto;

#[no_mangle]
pub extern "C" fn call_service(_message_char: *const c_char) -> *mut c_char {
    let message = r#"{"is_success":true,"result":{"type":"PEER","command":"CREATE","peer_id":"data_caller","token":"pt-7dceefb0-5e34-4dc4-a433-3c8b56345247"}}"#;
    return CString::new(message).unwrap().into_raw();
}

#[no_mangle]
pub extern "C" fn receive_events() -> *mut c_char {
    let message = r#"{"is_success":true,"result":{"type":"DATA","command":"REDIRECT","data_connection_id":"dc-91769c5a-a3f1-442d-981f-fc19b4875fd6"}}"#;
    return CString::new(message).unwrap().into_raw();
}
