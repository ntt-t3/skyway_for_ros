// skywayプロジェクト全体をDomainモデルとして整理すると
// rust_module以下はApplicationの一部とDomain、Infra層に相当する
// skyway_webrtc_gateway_controller crate(以下SkyWay Crate)をInfra層として利用し、
// ROS側で持つべきDomain知識を定義し、サービスを提供するのが主な目的である

mod domain;
mod error;
mod infra;

use std::ffi::CString;
use std::os::raw::c_char;

use once_cell::sync::OnceCell;

#[derive(Debug)]
pub struct Logger {
    pub debug_c: extern "C" fn(*const c_char) -> (),
    pub info_c: extern "C" fn(*const c_char) -> (),
    pub warn_c: extern "C" fn(*const c_char) -> (),
    pub error_c: extern "C" fn(*const c_char) -> (),
}

impl Logger {
    pub fn global() -> &'static Logger {
        INSTANCE.get().expect("logger is not initialized")
    }

    fn debug(&self, message: impl Into<String>) {
        let message_raw = CString::new(message.into()).unwrap().into_raw();
        (self.debug_c)(message_raw);
    }

    fn info(&self, message: impl Into<String>) {
        let message_raw = CString::new(message.into()).unwrap().into_raw();
        (self.info_c)(message_raw);
    }

    fn warn(&self, message: impl Into<String>) {
        let message_raw = CString::new(message.into()).unwrap().into_raw();
        (self.warn_c)(message_raw);
    }

    fn error(&self, message: impl Into<String>) {
        let message_raw = CString::new(message.into()).unwrap().into_raw();
        (self.error_c)(message_raw);
    }
}

static INSTANCE: OnceCell<Logger> = OnceCell::new();

#[no_mangle]
pub extern "C" fn register_logger(
    debug_c: extern "C" fn(*const c_char),
    info_c: extern "C" fn(*const c_char),
    warn_c: extern "C" fn(*const c_char),
    error_c: extern "C" fn(*const c_char),
) {
    INSTANCE
        .set(Logger {
            debug_c,
            info_c,
            warn_c,
            error_c,
        })
        .unwrap();

    let logger = Logger::global();
    logger.debug("debug");
    logger.info("info");
    logger.warn("warn");
    logger.error("error");
}
