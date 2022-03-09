pub(crate) mod peer;

use async_trait::async_trait;

use crate::application::Dto;
use crate::domain::entity::Response;
use crate::Repository;
use crate::{error, Logger, ProgramState};

#[async_trait]
pub(crate) trait Service {
    async fn execute(
        &self,
        repository: &Box<dyn Repository>,
        program_state: &ProgramState,
        logger: &Logger,
        message: Dto,
    ) -> Result<Response, error::Error>;
}
