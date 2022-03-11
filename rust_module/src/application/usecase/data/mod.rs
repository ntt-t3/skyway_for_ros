pub(crate) mod connect;
pub(crate) mod redirect;

use crate::domain::entity::{
    DataId, DataRequestParams, DataResponseMessageBodyEnum, Request, Response,
    ResponseMessageBodyEnum, SerializableSocket,
};
use crate::{error, Logger, ProgramState, Repository};

async fn create_data(
    repository: &Box<dyn Repository>,
    program_state: &ProgramState,
    logger: &Logger,
) -> Result<(DataId, String, u16), error::Error> {
    logger.debug("create_data for DATA CONNECT");
    let request = Request::Data(DataRequestParams::Create { params: true });
    let result = repository.register(program_state, logger, request).await?;

    return match result {
        Response::Success(ResponseMessageBodyEnum::Data(DataResponseMessageBodyEnum::Create(
            ref socket_info,
        ))) => {
            let data_id = socket_info.get_id().unwrap();
            let address = socket_info.ip().to_string();
            let port = socket_info.port();
            Ok((data_id, address, port))
        }
        _ => Err(error::Error::create_local_error("invalid response")),
    };
}
