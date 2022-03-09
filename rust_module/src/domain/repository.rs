use async_trait::async_trait;

use crate::domain::entity::{Request, Response};
use crate::{error, Logger, ProgramState};

#[cfg(test)]
use mockall::automock;

#[cfg_attr(test, automock)]
#[async_trait]
pub(crate) trait Repository: Send + Sync {
    async fn register(
        &self,
        program_state: &ProgramState,
        logger: &Logger,
        params: Request,
    ) -> Result<Response, error::Error>;
    async fn receive_event(
        &self,
        program_state: &ProgramState,
        logger: &Logger,
    ) -> Result<Response, error::Error>;
}
