// skywayプロジェクト全体をDomainモデルとして整理すると
// rust_module以下はApplicationの一部とDomain、Infra層に相当する
// skyway_webrtc_gateway_controller crate(以下SkyWay Crate)をInfra層として利用し、
// ROS側で持つべきDomain知識を定義し、サービスを提供するのが主な目的である

mod application;
pub(crate) mod c_module;
mod domain;
mod error;
mod infra;

use std::ffi::{c_void, CString};
use std::os::raw::c_char;
use std::thread::JoinHandle;

use once_cell::sync::OnceCell;

use crate::c_module::*;
use crate::domain::repository::Repository;
use crate::infra::RepositoryImpl;

static REPOSITORY_INSTANCE: OnceCell<Box<dyn Repository>> = OnceCell::new();

#[repr(C)]
pub struct RunResponse {
    flag: bool,
    handler: *mut c_void,
}

#[no_mangle]
pub extern "C" fn run() -> RunResponse {
    if !Logger::is_allocated() {
        return RunResponse {
            flag: false,
            handler: std::ptr::null_mut(),
        };
    }

    if !ProgramState::is_allocated() {
        Logger::global().error(
            "ProgramState object is not allocated. Please call the register_program_state function",
        );
        return RunResponse {
            flag: false,
            handler: std::ptr::null_mut(),
        };
    }

    // SkyWay Crateを開始する
    let handle: JoinHandle<()> = std::thread::spawn(|| {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async {
            let (sender, receiver) = module::run("http://localhost:8000").await;
            // SkyWay Crateにアクセスするためのsender, receiverはRepositoryの中で保持する
            // Repositoryはonce_cellでglobalで確保される
            let repository = RepositoryImpl::new(sender, receiver);

            if REPOSITORY_INSTANCE.set(Box::new(repository)).is_err() {
                return;
            }
            ProgramState::global().wait_for_shutdown();
        });
    });

    let thread_handle = Box::into_raw(Box::new(handle)) as *mut c_void;

    return RunResponse {
        flag: true,
        handler: thread_handle,
    };
}

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
