use async_trait::async_trait;
use std::fmt::{Debug, Formatter};

use crate::domain::entity::{Request, Response};
use crate::error;

#[cfg(test)]
use mockall::automock;

#[cfg_attr(test, automock)]
#[async_trait]
pub(crate) trait Repository {
    async fn register(&self, params: Request) -> Result<Response, error::Error>;
    async fn receive_event(&self) -> Result<Response, error::Error>;
}

impl Debug for dyn Repository {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "Repository")
    }
}
