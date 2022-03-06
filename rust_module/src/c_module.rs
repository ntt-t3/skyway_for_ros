use std::ffi::CString;
use std::os::raw::{c_char, c_double};

use once_cell::sync::OnceCell;

#[derive(Debug)]
pub struct Logger {
    debug_c: extern "C" fn(*const c_char) -> (),
    info_c: extern "C" fn(*const c_char) -> (),
    warn_c: extern "C" fn(*const c_char) -> (),
    error_c: extern "C" fn(*const c_char) -> (),
}

#[allow(dead_code)]
impl Logger {
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

static LOGGER_INSTANCE: OnceCell<Logger> = OnceCell::new();

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
    pub fn global() -> &'static ProgramState {
        PROGRAM_STATE_INSTANCE
            .get()
            .expect("logger is not initialized")
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

static PROGRAM_STATE_INSTANCE: OnceCell<ProgramState> = OnceCell::new();

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
