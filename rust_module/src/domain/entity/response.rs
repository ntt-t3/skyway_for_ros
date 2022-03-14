use crate::error;

#[allow(dead_code)]
pub(crate) use module::prelude::response_message::{
    DataResponse, MediaResponse, PeerResponse, ResponseResult,
};

use crate::domain::entity::{FromStr, Stringify};
pub(crate) use module::prelude::response_message::ResponseMessage as Response;

impl Stringify for ResponseResult {
    fn to_string(&self) -> Result<String, error::Error> {
        return serde_json::to_string(self).map_err(|e| error::Error::SerdeError { error: e });
    }
}

impl FromStr for ResponseResult {
    fn from_str(raw_message: &str) -> Result<Self, error::Error> {
        serde_json::from_str(raw_message).map_err(|e| error::Error::SerdeError { error: e })
    }
}
