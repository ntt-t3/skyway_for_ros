pub(crate) mod event;
pub(crate) mod peer;

use async_trait::async_trait;
use shaku::Interface;

use crate::application::dto::request::RequestDto;
use crate::application::dto::response::ResponseDtoResult;
use crate::error;

#[cfg(test)]
use mockall::automock;

#[async_trait]
#[cfg_attr(test, automock)]
pub(crate) trait Service: Interface {
    async fn execute(&self, request: RequestDto) -> Result<ResponseDtoResult, error::Error>;
}
