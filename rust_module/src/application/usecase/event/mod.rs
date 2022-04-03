use std::sync::Arc;

use async_trait::async_trait;
use shaku::{Component, Interface};

use crate::application::dto::response::{
    DataConnectionEventDto, DataResponseDto, PeerResponseDto, ResponseDto, ResponseDtoResult,
};
use crate::domain::entity::response::{DataResponse, PeerResponse, Response, ResponseResult};
use crate::domain::entity::{DataConnectionEventEnum, MediaConnectionEventEnum, PeerEventEnum};
use crate::domain::repository::Repository;
use crate::error::Error;
use crate::utils::CallbackCaller;
use crate::{error, DataConnectionResponse, GlobalState};

#[cfg(test)]
use mockall::automock;

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
            ResponseResult::Success(Response::Peer(PeerResponse::Event(event))) => Ok(
                ResponseDtoResult::Success(ResponseDto::Peer(PeerResponseDto::Event(event))),
            ),
            ResponseResult::Success(Response::Data(DataResponse::Event(event))) => match event {
                DataConnectionEventEnum::OPEN(open) => {
                    if let Some(item) = self.state.find_topic(&open.data_connection_id) {
                        let response = DataConnectionResponse {
                            data_connection_id: open.data_connection_id,
                            source_topic_name: item.source_topic_name,
                            source_ip: item.source_ip,
                            source_port: item.source_port,
                            destination_topic_name: item.destination_topic_name,
                        };
                        Ok(ResponseDtoResult::Success(ResponseDto::Data(
                            DataResponseDto::Event(DataConnectionEventDto::OPEN(response)),
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
            _ => Err(error::Error::create_local_error(
                "invalid response for GenealService",
            )),
        }
    }
}
