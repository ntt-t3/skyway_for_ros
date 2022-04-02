use std::ffi::{c_void, CStr, CString};
use std::os::raw::c_char;
use std::thread::JoinHandle;

use crate::ffi::global_params::{Logger, ProgramState};

#[allow(dead_code)]
pub(crate) mod global_params {
    use std::ffi::CString;
    use std::os::raw::{c_char, c_double};

    use serde::{Deserialize, Serialize};

    use crate::domain::entity::DataConnectionId;
    use crate::ffi::TopicParameters;
    use crate::{CALLBACK_FUNCTIONS, LOGGER_INSTANCE, PROGRAM_STATE_INSTANCE};

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
        shutdown_c: extern "C" fn() -> (),
    }

    #[allow(dead_code)]
    impl ProgramState {
        pub fn new(
            is_running_c: extern "C" fn() -> bool,
            is_shutting_down_c: extern "C" fn() -> bool,
            sleep_c: extern "C" fn(c_double) -> (),
            wait_for_shutdown_c: extern "C" fn() -> (),
            shutdown_c: extern "C" fn() -> (),
        ) -> Self {
            ProgramState {
                is_running_c,
                is_shutting_down_c,
                sleep_c,
                wait_for_shutdown_c,
                shutdown_c,
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

        pub fn shutdown(&self) {
            (self.shutdown_c)();
        }
    }

    #[no_mangle]
    pub extern "C" fn register_program_state(
        is_running_c: extern "C" fn() -> bool,
        is_shutting_down_c: extern "C" fn() -> bool,
        sleep_c: extern "C" fn(c_double) -> (),
        wait_for_shutdown_c: extern "C" fn() -> (),
        shutdown_c: extern "C" fn() -> (),
    ) {
        PROGRAM_STATE_INSTANCE
            .set(ProgramState {
                is_running_c,
                is_shutting_down_c,
                sleep_c,
                wait_for_shutdown_c,
                shutdown_c,
            })
            .unwrap();
    }

    #[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
    pub(crate) struct DataConnectionResponse {
        pub data_connection_id: DataConnectionId,
        pub source_topic_name: String,
        pub source_ip: String,
        pub source_port: u16,
        pub destination_topic_name: String,
    }
}

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
            crate::rust_main().await;
        });
    });

    let thread_handle = Box::into_raw(Box::new(handle)) as *mut c_void;

    return RunResponse {
        flag: true,
        handler: thread_handle,
    };
}

#[repr(C)]
pub struct SourceParameters {
    pub(crate) source_topic_name: *mut c_char,
    pub(crate) destination_address: *mut c_char,
    pub(crate) destination_port: u16,
}

#[repr(C)]
pub struct DestinationParameters {
    pub(crate) source_port: u16,
    pub(crate) destination_topic_name: *mut c_char,
}

#[repr(C)]
pub struct TopicParameters {
    pub(crate) data_connection_id: *mut c_char,
    pub(crate) source_parameters: SourceParameters,
    pub(crate) destination_parameters: DestinationParameters,
}

// ========== application methods ==========

// 当面はユニットテストは行わず、結合試験だけ行うことにする
// Fixme: Unit Test
#[no_mangle]
pub extern "C" fn call_service(message_char: *const c_char) -> *mut c_char {
    // C文字列とRust文字列の変換だけ行って、中身の処理はapplicationメソッドに任せる
    let rt = tokio::runtime::Runtime::new().unwrap();
    let message: String = rt.block_on(async {
        let c_str: &CStr = unsafe { CStr::from_ptr(message_char) };
        let message = c_str.to_str().unwrap().to_string();

        crate::application::call_service(message).await
    });
    return CString::new(message.as_str()).unwrap().into_raw();
}

#[no_mangle]
pub extern "C" fn receive_events() -> *mut c_char {
    // C文字列とRust文字列の変換だけ行って、中身の処理はapplicationメソッドに任せる
    let rt = tokio::runtime::Runtime::new().unwrap();
    let result = rt.block_on(async { crate::application::receive_events().await });
    return CString::new(result).unwrap().into_raw();
}

#[no_mangle]
pub extern "C" fn shutdown_service(peer_id: *const c_char, token: *const c_char) {
    todo!()
}

#[cfg(test)]
pub(crate) mod helper {
    use std::os::raw::c_double;

    pub extern "C" fn is_running() -> bool {
        true
    }

    pub extern "C" fn is_shutting_down() -> bool {
        false
    }

    pub extern "C" fn sleep(_param: c_double) {}

    pub extern "C" fn wait_for_shutdown() {}

    pub extern "C" fn shutdown() {}
}
