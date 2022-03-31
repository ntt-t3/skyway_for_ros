use async_trait::async_trait;
use shaku::Interface;

use crate::domain::entity::response::ResponseResult;
use crate::error;

use crate::domain::entity::request::Request;
#[cfg(test)]
use mockall::automock;

#[cfg_attr(test, automock)]
#[async_trait]
pub(crate) trait Repository: Interface {
    async fn register(&self, params: Request) -> Result<ResponseResult, error::Error>;
    async fn receive_event(&self) -> Result<ResponseResult, error::Error>;
}
