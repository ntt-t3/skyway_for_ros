use crate::application::dto::response::{
    DataConnectionEventDto, DataResponseDto, MediaResponseDto, PeerResponseDto, ResponseDto,
    ResponseDtoResult,
};
use crate::application::{CallbackFunctions, ErrorMessage, ErrorMessageInternal};
use crate::domain::entity::response::{
    DataResponse, MediaResponse, PeerResponse, Response, ResponseResult,
};
use crate::domain::entity::{
    DataConnectionEventEnum, MediaConnectionEventEnum, PeerEventEnum, Stringify,
};
use crate::{get_data_connection_state, Repository};
use crate::{Logger, ProgramState};

pub(crate) struct Event {}

impl Default for Event {
    fn default() -> Self {
        Event {}
    }
}

fn peer_event(
    _repository: &Box<dyn Repository>,
    _program_state: &ProgramState,
    _logger: &Logger,
    cb_functions: &CallbackFunctions,
    result: ResponseResult,
) -> ResponseDtoResult {
    match result {
        ResponseResult::Success(Response::Peer(PeerResponse::Event(event))) => match event {
            PeerEventEnum::OPEN(open) => ResponseDtoResult::Success(ResponseDto::Peer(
                PeerResponseDto::Event(PeerEventEnum::OPEN(open)),
            )),
            PeerEventEnum::CLOSE(close) => {
                cb_functions.peer_deleted_callback();
                ResponseDtoResult::Success(ResponseDto::Peer(PeerResponseDto::Event(
                    PeerEventEnum::CLOSE(close),
                )))
            }
            PeerEventEnum::CALL(open) => ResponseDtoResult::Success(ResponseDto::Peer(
                PeerResponseDto::Event(PeerEventEnum::CALL(open)),
            )),
            PeerEventEnum::CONNECTION(connect) => ResponseDtoResult::Success(ResponseDto::Peer(
                PeerResponseDto::Event(PeerEventEnum::CONNECTION(connect)),
            )),
            PeerEventEnum::ERROR(error) => ResponseDtoResult::Success(ResponseDto::Peer(
                PeerResponseDto::Event(PeerEventEnum::ERROR(error)),
            )),
            PeerEventEnum::TIMEOUT => unreachable!(),
        },
        _ => unreachable!(),
    }
}

fn media_event(
    _repository: &Box<dyn Repository>,
    _program_state: &ProgramState,
    _logger: &Logger,
    _cb_functions: &CallbackFunctions,
    result: ResponseResult,
) -> ResponseDtoResult {
    match result {
        ResponseResult::Success(Response::Media(MediaResponse::Event(event))) => match event {
            MediaConnectionEventEnum::READY(ready) => {
                ResponseDtoResult::Success(ResponseDto::Media(MediaResponseDto::Event(
                    MediaConnectionEventEnum::READY(ready),
                )))
            }
            MediaConnectionEventEnum::STREAM(stream) => {
                ResponseDtoResult::Success(ResponseDto::Media(MediaResponseDto::Event(
                    MediaConnectionEventEnum::STREAM(stream),
                )))
            }
            MediaConnectionEventEnum::CLOSE(close) => {
                ResponseDtoResult::Success(ResponseDto::Media(MediaResponseDto::Event(
                    MediaConnectionEventEnum::CLOSE(close),
                )))
            }
            MediaConnectionEventEnum::ERROR(error) => {
                ResponseDtoResult::Success(ResponseDto::Media(MediaResponseDto::Event(
                    MediaConnectionEventEnum::ERROR(error),
                )))
            }
            MediaConnectionEventEnum::TIMEOUT => {
                unreachable!()
            }
        },
        _ => unreachable!(),
    }
}

fn data_event(
    _repository: &Box<dyn Repository>,
    _program_state: &ProgramState,
    _logger: &Logger,
    cb_functions: &CallbackFunctions,
    result: ResponseResult,
) -> ResponseDtoResult {
    match result {
        ResponseResult::Success(Response::Data(DataResponse::Event(event))) => match event {
            DataConnectionEventEnum::OPEN(open) => {
                match get_data_connection_state()
                    .lock()
                    .unwrap()
                    .get(&open.data_connection_id)
                {
                    Some(item) => ResponseDtoResult::Success(ResponseDto::Data(
                        DataResponseDto::Event(DataConnectionEventDto::OPEN(item.clone())),
                    )),
                    None => ResponseDtoResult::Error("unknown dataconnection".to_string()),
                }
            }
            DataConnectionEventEnum::CLOSE(close) => {
                cb_functions.data_connection_deleted_callback(close.data_connection_id.as_str());
                let _ = get_data_connection_state()
                    .lock()
                    .unwrap()
                    .remove(&close.data_connection_id);
                ResponseDtoResult::Success(ResponseDto::Data(DataResponseDto::Event(
                    DataConnectionEventDto::CLOSE(close),
                )))
            }
            DataConnectionEventEnum::ERROR(error) => ResponseDtoResult::Success(ResponseDto::Data(
                DataResponseDto::Event(DataConnectionEventDto::ERROR(error)),
            )),
            DataConnectionEventEnum::TIMEOUT => {
                unreachable!()
            }
        },
        _ => unreachable!(),
    }
}

// TODO: Unit Test
impl Event {
    pub async fn execute(
        &self,
        repository: &Box<dyn Repository>,
        program_state: &ProgramState,
        logger: &Logger,
        cb_functions: &CallbackFunctions,
    ) -> String {
        let event = repository.receive_event(program_state, logger).await;
        match event {
            Ok(inner) => match inner {
                ResponseResult::Success(Response::Peer(PeerResponse::Event(..))) => {
                    peer_event(repository, program_state, logger, cb_functions, inner)
                        .to_string()
                        .unwrap()
                }
                ResponseResult::Success(Response::Media(MediaResponse::Event(..))) => {
                    media_event(repository, program_state, logger, cb_functions, inner)
                        .to_string()
                        .unwrap()
                }
                ResponseResult::Success(Response::Data(DataResponse::Event(..))) => {
                    data_event(repository, program_state, logger, cb_functions, inner)
                        .to_string()
                        .unwrap()
                }
                _ => unreachable!(),
            },
            Err(e) => {
                let internal = ErrorMessageInternal {
                    r#type: Some("EVENT".to_string()),
                    command: Some("LISTEN".to_string()),
                    error: e.to_string(),
                };
                let error_message = ErrorMessage {
                    is_success: false,
                    result: internal,
                };
                // to_stringはserde_jsonのSerializeしているだけで、型定義により確実に成功するのでunwrapしてよい
                error_message.to_string().unwrap()
            }
        }
    }
}
