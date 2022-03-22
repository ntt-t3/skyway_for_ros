use crate::domain::entity::request::{IsVideo, MediaRequest, Request};
use crate::domain::entity::response::{MediaResponse, Response, ResponseResult};
use crate::domain::entity::{MediaId, RtcpId, SocketInfo};
use crate::{error, Logger, ProgramState, Repository};

pub(crate) mod answer;
pub(crate) mod call;

async fn create_media(
    repository: &Box<dyn Repository>,
    program_state: &ProgramState,
    logger: &Logger,
    is_video: bool,
) -> Result<SocketInfo<MediaId>, error::Error> {
    let message = Request::Media(MediaRequest::ContentCreate {
        params: IsVideo { is_video },
    });
    match repository.register(program_state, logger, message).await {
        Ok(ResponseResult::Success(Response::Media(MediaResponse::ContentCreate(socket)))) => {
            Ok(socket)
        }
        Err(e) => Err(e),
        _ => Err(error::Error::create_local_error(
            "error in create Video Object",
        )),
    }
}

async fn create_rtcp(
    repository: &Box<dyn Repository>,
    program_state: &ProgramState,
    logger: &Logger,
    param: Option<()>,
) -> Result<SocketInfo<RtcpId>, error::Error> {
    let message = Request::Media(MediaRequest::RtcpCreate { params: param });
    match repository.register(program_state, logger, message).await {
        Ok(ResponseResult::Success(Response::Media(MediaResponse::RtcpCreate(socket)))) => {
            Ok(socket)
        }
        Err(e) => Err(e),
        _ => Err(error::Error::create_local_error(
            "error in create Video Object",
        )),
    }
}
