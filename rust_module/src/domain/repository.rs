use async_trait::async_trait;

use crate::domain::entity::response::ResponseResult;
use crate::{error, Logger, ProgramState};

use crate::domain::entity::request::Request;
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
    ) -> Result<ResponseResult, error::Error>;
    async fn receive_event(
        &self,
        program_state: &ProgramState,
        logger: &Logger,
    ) -> Result<ResponseResult, error::Error>;
}
