use std::sync::Arc;
use std::thread::sleep;
use std::time::Duration;

use async_trait::async_trait;
use shaku::{Component, HasComponent, Interface};

use crate::application::dto::response::{
    DataConnectionEventDto, DataResponseDto, MediaConnectionEventEnumDto, MediaPair,
    MediaResponseDto, PeerResponseDto, ResponseDto, ResponseDtoResult, SendParams,
};
use crate::di::*;
use crate::domain::entity::response::{DataResponse, PeerResponse, Response, ResponseResult};
use crate::domain::entity::{DataConnectionEventEnum, MediaConnectionEventEnum, PeerEventEnum};
use crate::domain::repository::Repository;
use crate::error::Error;
use crate::utils::CallbackCaller;
use crate::{error, CallResponseDto, DataPipeInfo, GlobalState, Logger, ProgramState};

#[cfg(test)]
use mockall::automock;
use skyway_webrtc_gateway_caller::prelude::data::DataConnectionIdWrapper;
use skyway_webrtc_gateway_caller::prelude::response_parser::MediaResponse;

#[async_trait]
#[cfg_attr(test, automock)]
pub(crate) trait OnEvent: Interface {
    async fn execute(&self, event: Response) -> Result<ResponseResult, error::Error>;
}

#[async_trait]
#[cfg_attr(test, automock)]
pub(crate) trait EventReceive: Interface {
    async fn execute(&self) -> Result<ResponseDtoResult, error::Error>;
}

#[derive(Component)]
#[shaku(interface = EventReceive)]
pub(crate) struct EventReceiveImpl {
    #[shaku(inject)]
    logger: Arc<dyn Logger>,
    #[shaku(inject)]
    repository: Arc<dyn Repository>,
    #[shaku(inject)]
    state: Arc<dyn GlobalState>,
    #[shaku(inject)]
    callback: Arc<dyn CallbackCaller>,
}

#[derive(Debug, PartialEq)]
pub(crate) enum EventEnum {
    Peer(PeerEventEnum),
    Data(DataConnectionEventEnum),
    Media(MediaConnectionEventEnum),
}

#[async_trait]
impl EventReceive for EventReceiveImpl {
    async fn execute(&self) -> Result<ResponseDtoResult, Error> {
        let event = self.repository.receive_event().await?;
        self.result_to_dto(event)
    }
}

impl EventReceiveImpl {
    fn result_to_dto(&self, response: ResponseResult) -> Result<ResponseDtoResult, error::Error> {
        match response {
            ResponseResult::Success(Response::Peer(PeerResponse::Event(PeerEventEnum::CLOSE(
                close,
            )))) => {
                std::thread::spawn(|| {
                    sleep(Duration::from_millis(100));
                    let module = CppObjctsModule::builder().build();
                    let state: &dyn ProgramState = module.resolve_ref();
                    state.shutdown();
                });

                Ok(ResponseDtoResult::Success(ResponseDto::Peer(
                    PeerResponseDto::Event(PeerEventEnum::CLOSE(close)),
                )))
            }
            ResponseResult::Success(Response::Peer(PeerResponse::Event(event))) => Ok(
                ResponseDtoResult::Success(ResponseDto::Peer(PeerResponseDto::Event(event))),
            ),
            ResponseResult::Success(Response::Data(DataResponse::Event(event))) => match event {
                DataConnectionEventEnum::OPEN(open) => {
                    if let Some(item) = self.state.find_topic(&open.data_connection_id) {
                        Ok(ResponseDtoResult::Success(ResponseDto::Data(
                            DataResponseDto::Event(DataConnectionEventDto::OPEN(
                                DataConnectionIdWrapper {
                                    data_connection_id: item.data_connection_id,
                                },
                            )),
                        )))
                    } else {
                        let message = format!(
                            "no info about DataConnectionId {:?}",
                            open.data_connection_id.as_str()
                        );
                        Err(error::Error::create_local_error(&message))
                    }
                }
                DataConnectionEventEnum::CLOSE(close) => {
                    self.state.remove_topic(&close.data_connection_id);
                    self.callback
                        .data_connection_deleted_callback(close.data_connection_id.as_str());

                    Ok(ResponseDtoResult::Success(ResponseDto::Data(
                        DataResponseDto::Event(DataConnectionEventDto::CLOSE(close)),
                    )))
                }
                _ => {
                    todo!()
                }
            },
            ResponseResult::Success(Response::Media(MediaResponse::Event(event))) => match event {
                MediaConnectionEventEnum::STREAM(stream) => {
                    let response = self
                        .state
                        .find_call_response(&stream.media_connection_id)
                        .expect("call response info is not stored");

                    let call_response_dto = CallResponseDto {
                        send_params: response.send_params,
                        redirect_params: response.redirect_params,
                        media_connection_id: stream.media_connection_id,
                    };
                    Ok(ResponseDtoResult::Success(ResponseDto::Media(
                        MediaResponseDto::Event(MediaConnectionEventEnumDto::Stream(
                            call_response_dto,
                        )),
                    )))
                }
                MediaConnectionEventEnum::READY(stream) => {
                    let response = self
                        .state
                        .find_call_response(&stream.media_connection_id)
                        .expect("call response info is not stored");

                    let call_response_dto = CallResponseDto {
                        send_params: response.send_params,
                        redirect_params: response.redirect_params,
                        media_connection_id: stream.media_connection_id,
                    };
                    Ok(ResponseDtoResult::Success(ResponseDto::Media(
                        MediaResponseDto::Event(MediaConnectionEventEnumDto::Ready(
                            call_response_dto,
                        )),
                    )))
                }
                MediaConnectionEventEnum::CLOSE(id_wrapper) => {
                    Ok(ResponseDtoResult::Success(ResponseDto::Media(
                        MediaResponseDto::Event(MediaConnectionEventEnumDto::Close(id_wrapper)),
                    )))
                }
                _ => todo!(),
            },
            event => {
                let message = format!("event {:?} is not covered", event);
                self.logger.warn(&message);
                todo!()
            }
        }
    }
}
