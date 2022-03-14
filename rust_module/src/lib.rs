// skywayプロジェクト全体をDomainモデルとして整理すると
// rust_module以下はApplicationの一部とDomain、Infra層に相当する
// skyway_webrtc_gateway_controller crate(以下SkyWay Crate)をInfra層として利用し、
// ROS側で持つべきDomain知識を定義し、サービスを提供するのが主な目的である

mod application;
mod domain;
mod error;
mod infra;

use std::ffi::{c_void, CString};
use std::os::raw::{c_char, c_double};
use std::thread::JoinHandle;

use once_cell::sync::OnceCell;

use crate::application::TopicParameters;
use crate::domain::repository::Repository;
use crate::infra::RepositoryImpl;

// C++側とオブジェクトのやり取りをする回数を最低限にするため、C++側のモジュールで本来所持するべきオブジェクトはOnceCellで保持する
//========== ↓ OnceCell ↓ ==========
// C++側に返すべきコールバックを保持する
static CALLBACK_FUNCTIONS: OnceCell<CallbackFunctions> = OnceCell::new();
// Log出力するために必要なC++側の関数を保持する
static LOGGER_INSTANCE: OnceCell<Logger> = OnceCell::new();
// Programの状態を取得・操作するために必要なC++側の関数を保持する
static PROGRAM_STATE_INSTANCE: OnceCell<ProgramState> = OnceCell::new();
// RepositoryとしてWebRTC Crateを利用しているが、生成されたSender, Receiverを破棄すると通信できなくなるので、保持し続ける
static REPOSITORY_INSTANCE: OnceCell<Box<dyn Repository>> = OnceCell::new();

#[repr(C)]
pub struct CallbackFunctions {
    create_peer_callback_c: extern "C" fn(peer_id: *mut c_char, token: *mut c_char),
    peer_deleted_callback: extern "C" fn(),
    data_callback_c: extern "C" fn(param: TopicParameters),
    data_connection_deleted_callback_c: extern "C" fn(data_connection_id: *mut c_char),
}

impl CallbackFunctions {
    pub fn new(
        create_peer_callback_c: extern "C" fn(peer_id: *mut c_char, token: *mut c_char),
        peer_deleted_callback: extern "C" fn(),
        data_callback_c: extern "C" fn(param: TopicParameters),
        data_connection_deleted_callback_c: extern "C" fn(data_connection_id: *mut c_char),
    ) -> Self {
        CallbackFunctions {
            create_peer_callback_c,
            peer_deleted_callback,
            data_callback_c,
            data_connection_deleted_callback_c,
        }
    }

    pub fn global() -> &'static CallbackFunctions {
        CALLBACK_FUNCTIONS
            .get()
            .expect("functions is not initialized")
    }

    pub fn create_peer_callback(&self, peer_id: &str, token: &str) {
        (self.create_peer_callback_c)(
            CString::new(peer_id).unwrap().into_raw(),
            CString::new(token).unwrap().into_raw(),
        );
    }

    pub fn peer_deleted_callback(&self) {
        (self.peer_deleted_callback)();
    }

    pub fn data_callback(&self, param: TopicParameters) {
        (self.data_callback_c)(param);
    }

    pub fn data_connection_deleted_callback(&self, data_connection_id: &str) {
        (self.data_connection_deleted_callback_c)(
            CString::new(data_connection_id).unwrap().into_raw(),
        );
    }
}

#[no_mangle]
pub extern "C" fn register_callbacks(param: &CallbackFunctions) {
    let functions = CallbackFunctions {
        create_peer_callback_c: param.create_peer_callback_c,
        peer_deleted_callback: param.peer_deleted_callback,
        data_callback_c: param.data_callback_c,
        data_connection_deleted_callback_c: param.data_connection_deleted_callback_c,
    };

    if CALLBACK_FUNCTIONS.set(functions).is_err() {
        return;
    }
}

#[derive(Debug)]
pub struct Logger {
    debug_c: extern "C" fn(*const c_char) -> (),
    info_c: extern "C" fn(*const c_char) -> (),
    warn_c: extern "C" fn(*const c_char) -> (),
    error_c: extern "C" fn(*const c_char) -> (),
}

#[allow(dead_code)]
impl Logger {
    pub fn new(
        debug_c: extern "C" fn(*const c_char) -> (),
        info_c: extern "C" fn(*const c_char) -> (),
        warn_c: extern "C" fn(*const c_char) -> (),
        error_c: extern "C" fn(*const c_char) -> (),
    ) -> Self {
        Logger {
            debug_c,
            info_c,
            warn_c,
            error_c,
        }
    }

    pub fn global() -> &'static Logger {
        LOGGER_INSTANCE.get().expect("logger is not initialized")
    }

    pub fn is_allocated() -> bool {
        LOGGER_INSTANCE.get().is_some()
    }

    pub fn debug(&self, message: impl Into<String>) {
        let message_raw = CString::new(message.into()).unwrap().into_raw();
        (self.debug_c)(message_raw);
    }

    pub fn info(&self, message: impl Into<String>) {
        let message_raw = CString::new(message.into()).unwrap().into_raw();
        (self.info_c)(message_raw);
    }

    pub fn warn(&self, message: impl Into<String>) {
        let message_raw = CString::new(message.into()).unwrap().into_raw();
        (self.warn_c)(message_raw);
    }

    pub fn error(&self, message: impl Into<String>) {
        let message_raw = CString::new(message.into()).unwrap().into_raw();
        (self.error_c)(message_raw);
    }
}

#[no_mangle]
pub extern "C" fn register_logger(
    debug_c: extern "C" fn(*const c_char),
    info_c: extern "C" fn(*const c_char),
    warn_c: extern "C" fn(*const c_char),
    error_c: extern "C" fn(*const c_char),
) {
    LOGGER_INSTANCE
        .set(Logger {
            debug_c,
            info_c,
            warn_c,
            error_c,
        })
        .unwrap();
}

#[derive(Debug)]
// このモジュールでは基本的にROSの制御を指す
pub struct ProgramState {
    is_running_c: extern "C" fn() -> bool,
    is_shutting_down_c: extern "C" fn() -> bool,
    sleep_c: extern "C" fn(c_double) -> (),
    wait_for_shutdown_c: extern "C" fn() -> (),
}

#[allow(dead_code)]
impl ProgramState {
    pub fn new(
        is_running_c: extern "C" fn() -> bool,
        is_shutting_down_c: extern "C" fn() -> bool,
        sleep_c: extern "C" fn(c_double) -> (),
        wait_for_shutdown_c: extern "C" fn() -> (),
    ) -> Self {
        ProgramState {
            is_running_c,
            is_shutting_down_c,
            sleep_c,
            wait_for_shutdown_c,
        }
    }

    pub fn global() -> &'static ProgramState {
        PROGRAM_STATE_INSTANCE
            .get()
            .expect("ProgramState is not initialized")
    }

    pub fn is_allocated() -> bool {
        PROGRAM_STATE_INSTANCE.get().is_some()
    }

    pub fn is_running(&self) -> bool {
        (self.is_running_c)()
    }

    pub fn is_shutting_down(&self) -> bool {
        (self.is_shutting_down_c)()
    }

    pub fn sleep_c(&self, duration: f64) {
        (self.sleep_c)(duration as c_double);
    }

    pub fn wait_for_shutdown(&self) {
        (self.wait_for_shutdown_c)();
    }
}

#[no_mangle]
pub extern "C" fn register_program_state(
    is_running_c: extern "C" fn() -> bool,
    is_shutting_down_c: extern "C" fn() -> bool,
    sleep_c: extern "C" fn(c_double) -> (),
    wait_for_shutdown_c: extern "C" fn() -> (),
) {
    PROGRAM_STATE_INSTANCE
        .set(ProgramState {
            is_running_c,
            is_shutting_down_c,
            sleep_c,
            wait_for_shutdown_c,
        })
        .unwrap();
}

//========== ↑ OnceCell ↑ ==========

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
