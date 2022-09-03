pub(crate) mod dto;
pub(crate) mod factory;
pub(crate) mod usecase;

use serde::{Deserialize, Serialize};
use shaku::HasComponent;

use crate::application::dto::request::RequestDto;
use crate::application::dto::Command;
use crate::application::factory::Factory;
use crate::application::usecase::event::EventReceive;
use crate::di::*;
use crate::domain::entity::Stringify;
use crate::error;
use crate::ffi::rust_to_c_bridge::c_functions_wrapper::LoggerHolder;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
struct ErrorMessage {
    is_success: bool,
    result: ErrorMessageInternal,
}

impl Stringify for ErrorMessage {
    fn to_string(&self) -> Result<String, error::Error> {
        serde_json::to_string(self).map_err(|e| error::Error::SerdeError { error: e })
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
struct ErrorMessageInternal {
    r#type: Option<String>,
    command: Option<String>,
    error: String,
}

// called from ffi::call_service
pub(crate) async fn call_service(message: String) -> String {
    match RequestDto::from_str(&message) {
        Ok(dto) => {
            let module = GeneralFactory::builder().build();
            let factory: &dyn Factory = module.resolve_ref();
            let service = factory.create_service(&dto);

            // errorメッセージを生成する際に必要なので確保しておく
            let command = dto.command();
            let dto_type = dto.dto_type();

            match service.execute(dto).await {
                Ok(response) => {
                    // ResponseMessageはto_stringでエラーを出すことはない
                    response.to_string().unwrap()
                }
                Err(e) => {
                    let internal = ErrorMessageInternal {
                        r#type: Some(dto_type),
                        command: Some(command),
                        error: format!("{:?}", e),
                    };
                    let error_message = ErrorMessage {
                        is_success: false,
                        result: internal,
                    };
                    error_message.to_string().unwrap()
                }
            }
        }
        Err(_e) => {
            let internal = ErrorMessageInternal {
                r#type: None,
                command: None,
                error: format!("invalid message: {}", message),
            };
            let error_message = ErrorMessage {
                is_success: false,
                result: internal,
            };
            let message = error_message.to_string().unwrap();
            LoggerHolder::global().error(message.as_str());
            message
        }
    }
}

// called from ffi::receive_events
pub async fn receive_events() -> String {
    let module = EventReceiveService::builder().build();
    let service: &dyn EventReceive = module.resolve_ref();
    let event = service.execute().await;
    match event {
        Ok(event) => serde_json::to_string(&event).unwrap(),
        Err(error) => {
            let internal = ErrorMessageInternal {
                r#type: None,
                command: None,
                error: format!("invalid message: {:?}", error),
            };
            let error_message = ErrorMessage {
                is_success: false,
                result: internal,
            };
            let message = error_message.to_string().unwrap();
            LoggerHolder::global().error(message.as_str());
            return message;
        }
    }
}
