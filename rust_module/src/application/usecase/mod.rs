pub(crate) mod create_peer;

use async_trait::async_trait;

use crate::application::Dto;
use crate::domain::entity::Response;
use crate::error;
use crate::Repository;

#[async_trait]
pub(crate) trait Service {
    async fn execute(
        &self,
        repository: &Box<dyn Repository>,
        message: Dto,
    ) -> Result<Response, error::Error>;
}
