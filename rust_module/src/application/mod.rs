mod dto;
mod usecase;

use std::ffi::{CStr, CString};
use std::os::raw::c_char;

use serde::{Deserialize, Serialize};

use crate::application::dto::{Command, Dto};
use crate::application::usecase::create_peer::CreatePeer;
use crate::application::usecase::Service;
use crate::domain::entity::{PeerServiceParams, Stringify};
use crate::error;
use crate::error::Error;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
struct ErrorMessage {
    is_success: bool,
    result: ErrorMessageInternal,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
struct ErrorMessageInternal {
    r#type: Option<String>,
    command: Option<String>,
    error: String,
}

impl Stringify for ErrorMessage {
    fn to_string(&self) -> Result<String, Error> {
        serde_json::to_string(self).map_err(|e| error::Error::SerdeError { error: e })
    }
}

// 当面はユニットテストは行わず、結合試験だけ行うことにする
// Fixme: Unit Test
#[no_mangle]
pub extern "C" fn call_service(message_char: *const c_char) -> *mut c_char {
    let rt = tokio::runtime::Runtime::new().unwrap();
    let message: String = rt.block_on(async {
        let c_str: &CStr = unsafe { CStr::from_ptr(message_char) };
        let message = c_str.to_str().unwrap().to_string();
        match Dto::from_str(&message) {
            Ok(dto) => {
                let repository = crate::REPOSITORY_INSTANCE.get().unwrap();
                let service = peer_factory(&dto);
                // errorメッセージを生成する際に必要なので確保しておく
                let command = dto.command();
                let dto_type = dto.dto_type();
                match service.execute(&repository, dto).await {
                    Ok(response) => {
                        // ResponseMessageはto_stringでエラーを出すことはない
                        response.to_string().unwrap()
                    }
                    Err(e) => {
                        let internal = ErrorMessageInternal {
                            r#type: Some(dto_type),
                            command: Some(command),
                            error: e.to_string(),
                        };
                        let error_message = ErrorMessage {
                            is_success: false,
                            result: internal,
                        };
                        error_message.to_string().unwrap()
                    }
                }
            }
            Err(e) => {
                let internal = ErrorMessageInternal {
                    r#type: None,
                    command: None,
                    error: e.to_string(),
                };
                let error_message = ErrorMessage {
                    is_success: false,
                    result: internal,
                };
                error_message.to_string().unwrap()
            }
        }
    });

    return CString::new(message.as_str()).unwrap().into_raw();
}

fn peer_factory(dto: &Dto) -> Box<dyn Service> {
    match dto {
        Dto::Peer(PeerServiceParams::Create {
            params: ref _params,
        }) => {
            let create_peer = CreatePeer {};
            Box::new(create_peer)
        }
        _ => {
            todo!()
        }
    }
}

#[no_mangle]
pub extern "C" fn receive_events() -> *mut c_char {
    todo!()
}
