mod dto;
pub(crate) mod usecase;

use std::ffi::{CStr, CString};
use std::os::raw::c_char;

use dto::request::{DataRequestDto, RequestDto};
use serde::{Deserialize, Serialize};

use crate::application::dto::request::PeerRequestDto;
use crate::application::dto::Command;
use crate::application::usecase::event::Event;
use crate::application::usecase::Service;
use crate::domain::entity::*;
use crate::{error, CallbackFunctions, Logger, ProgramState};
use usecase::peer;

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
    fn to_string(&self) -> Result<String, error::Error> {
        serde_json::to_string(self).map_err(|e| error::Error::SerdeError { error: e })
    }
}

#[repr(C)]
pub struct SourceParameters {
    source_topic_name: *mut c_char,
    destination_address: *mut c_char,
    destination_port: u16,
}

#[repr(C)]
pub struct DestinationParameters {
    source_port: u16,
    destination_topic_name: *mut c_char,
}

#[repr(C)]
pub struct TopicParameters {
    data_connection_id: *mut c_char,
    source_parameters: SourceParameters,
    destination_parameters: DestinationParameters,
}

// 当面はユニットテストは行わず、結合試験だけ行うことにする
// Fixme: Unit Test
#[no_mangle]
pub extern "C" fn call_service(message_char: *const c_char) -> *mut c_char {
    let rt = tokio::runtime::Runtime::new().unwrap();
    let message: String = rt.block_on(async {
        let c_str: &CStr = unsafe { CStr::from_ptr(message_char) };
        let message = c_str.to_str().unwrap().to_string();
        match RequestDto::from_str(&message) {
            Ok(dto) => {
                let repository = crate::REPOSITORY_INSTANCE.get().unwrap();
                let service = factory(&dto);
                // errorメッセージを生成する際に必要なので確保しておく
                let command = dto.command();
                let dto_type = dto.dto_type();
                match service
                    .execute(
                        &repository,
                        ProgramState::global(),
                        Logger::global(),
                        CallbackFunctions::global(),
                        dto,
                    )
                    .await
                {
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
                let message = error_message.to_string().unwrap();
                Logger::global().error(message.as_str());
                message
            }
        }
    });

    return CString::new(message.as_str()).unwrap().into_raw();
}

fn factory(dto: &RequestDto) -> Box<dyn Service> {
    let message = format!(
        "creating service in factory {}:{}",
        dto.dto_type(),
        dto.command()
    );
    Logger::global().debug(message);
    match dto {
        RequestDto::Peer(..) => peer_factory(dto),
        RequestDto::Data(..) => data_factory(dto),
        _ => Box::new(usecase::General {}),
    }
}

fn peer_factory(dto: &RequestDto) -> Box<dyn Service> {
    let message = format!(
        "creating service in peer_factory {}:{}",
        dto.dto_type(),
        dto.command()
    );
    Logger::global().debug(message);
    match dto {
        RequestDto::Peer(PeerRequestDto::Create {
            params: ref _params,
        }) => Box::new(peer::create::Create {}),
        _ => Box::new(usecase::General {}),
    }
}

fn data_factory(dto: &RequestDto) -> Box<dyn Service> {
    let message = format!(
        "creating service in data_factory {}:{}",
        dto.dto_type(),
        dto.command()
    );
    Logger::global().debug(message);
    match dto {
        RequestDto::Data(DataRequestDto::Connect { .. }) => {
            Box::new(usecase::data::connect::Connect::default())
        }
        RequestDto::Data(DataRequestDto::Redirect { .. }) => {
            Box::new(usecase::data::redirect::Redirect::default())
        }
        _ => Box::new(usecase::General {}),
    }
}

// FIXME: should be service
#[no_mangle]
pub extern "C" fn receive_events() -> *mut c_char {
    let rt = tokio::runtime::Runtime::new().unwrap();
    let repository = crate::REPOSITORY_INSTANCE.get().unwrap();
    let result = rt.block_on(async {
        let event = Event::default();
        event
            .execute(
                repository,
                ProgramState::global(),
                Logger::global(),
                CallbackFunctions::global(),
            )
            .await
    });

    return CString::new(result).unwrap().into_raw();
}

#[no_mangle]
pub extern "C" fn shutdown_service(peer_id: *const c_char, token: *const c_char) {
    let rt = tokio::runtime::Runtime::new().unwrap();
    rt.block_on(async {
        let c_str: &CStr = unsafe { CStr::from_ptr(peer_id) };
        let peer_id = c_str.to_str().unwrap().to_string();

        let c_str: &CStr = unsafe { CStr::from_ptr(token) };
        let token = c_str.to_str().unwrap().to_string();

        let message = format!(
            r#"{{
            "type": "PEER",
            "command": "DELETE",
            "params": {{
                "peer_id": "{}",
                "token": "{}"
            }}
        }}"#,
            peer_id, token
        );
        let param = RequestDto::from_str(&message).unwrap();
        let service = usecase::General {};
        let repository = crate::REPOSITORY_INSTANCE.get().unwrap();
        if let Err(e) = service
            .execute(
                &repository,
                ProgramState::global(),
                Logger::global(),
                CallbackFunctions::global(),
                param,
            )
            .await
        {
            let error_message = format!("peer close error: {:?}", e);
            Logger::global().error(error_message);
        }

        CallbackFunctions::global().peer_deleted_callback();
    });
}
