use async_trait::async_trait;
use shaku::Interface;

use crate::domain::entity::{Request, Response};
use crate::error;

#[cfg(test)]
use mockall::automock;

#[cfg_attr(test, automock)]
#[async_trait]
pub(crate) trait Repository: Interface {
    async fn register(&self, params: Request) -> Result<Response, error::Error>;
}
