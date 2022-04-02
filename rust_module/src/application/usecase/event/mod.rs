use std::sync::Arc;

use async_trait::async_trait;
use shaku::{Component, Interface};

use crate::domain::entity::response::{Response, ResponseResult};
use crate::domain::entity::{DataConnectionEventEnum, MediaConnectionEventEnum, PeerEventEnum};
use crate::domain::repository::Repository;
use crate::error::Error;
use crate::{error, GlobalState};

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
    async fn execute(&self) -> Result<ResponseResult, error::Error>;
}

#[derive(Component)]
#[shaku(interface = EventReceive)]
pub(crate) struct EventReceiveImpl {
    #[shaku(inject)]
    repository: Arc<dyn Repository>,
    #[shaku(inject)]
    state: Arc<dyn GlobalState>,
}

#[derive(Debug, PartialEq)]
pub(crate) enum EventEnum {
    Peer(PeerEventEnum),
    Data(DataConnectionEventEnum),
    Media(MediaConnectionEventEnum),
}

#[async_trait]
impl EventReceive for EventReceiveImpl {
    async fn execute(&self) -> Result<ResponseResult, Error> {
        self.repository.receive_event().await
    }
}
