use crate::error;

pub(crate) use module::prelude::request_message::PeerServiceParams;
pub(crate) use module::prelude::response_message::{
    PeerResponseMessageBodyEnum, ResponseMessageBodyEnum,
};
pub(crate) use module::prelude::*;
pub(crate) use module::ResponseMessage as Response;
pub(crate) use module::ServiceParams as Request;

// メッセージを自然にStringに変換できるようにする
pub(crate) trait Stringify {
    fn to_string(&self) -> Result<String, error::Error>;
}

// 自然にStrからメッセージに変換できるようにする
pub(crate) trait FromStr: Sized {
    fn from_str(raw_message: &str) -> Result<Self, error::Error>;
}

impl Stringify for Request {
    fn to_string(&self) -> Result<String, error::Error> {
        return serde_json::to_string(self).map_err(|e| error::Error::SerdeError { error: e });
    }
}

impl FromStr for Request {
    fn from_str(raw_message: &str) -> Result<Self, error::Error> {
        serde_json::from_str(raw_message).map_err(|e| error::Error::SerdeError { error: e })
    }
}

impl Stringify for Response {
    fn to_string(&self) -> Result<String, error::Error> {
        return serde_json::to_string(self).map_err(|e| error::Error::SerdeError { error: e });
    }
}

impl FromStr for Response {
    fn from_str(raw_message: &str) -> Result<Self, error::Error> {
        serde_json::from_str(raw_message).map_err(|e| error::Error::SerdeError { error: e })
    }
}
