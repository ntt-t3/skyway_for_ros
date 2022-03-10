use serde::ser::SerializeStruct;
use serde::{Deserialize, Serialize, Serializer};
use serde_json::Value;

use crate::error;
pub(crate) use module::prelude::*;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "command")]
pub enum PeerRequestParams {
    #[serde(rename = "CREATE")]
    Create { params: CreatePeerParams },
    #[serde(rename = "STATUS")]
    Status { params: PeerInfo },
    #[serde(rename = "DELETE")]
    Delete { params: PeerInfo },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "command")]
pub enum DataRequestParams {
    #[serde(rename = "CREATE")]
    Create,
    #[serde(rename = "DELETE")]
    Delete { params: DataIdWrapper },
    #[serde(rename = "CONNECT")]
    Connect { params: ConnectQuery },
    #[serde(rename = "REDIRECT")]
    Redirect { params: RedirectParams },
    #[serde(rename = "DISCONNECT")]
    Disconnect { params: DataConnectionIdWrapper },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "type")]
pub enum Request {
    #[serde(rename = "PEER")]
    Peer(PeerRequestParams),
    #[serde(rename = "DATA")]
    Data(DataRequestParams),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "command")]
pub enum PeerResponseMessageBodyEnum {
    #[serde(rename = "CREATE")]
    Create(PeerInfo),
    #[serde(rename = "STATUS")]
    Status(PeerStatusMessage),
    #[serde(rename = "DELETE")]
    Delete(PeerInfo),
    #[serde(rename = "EVENT")]
    Event(PeerEventEnum),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "command")]
pub enum DataResponseMessageBodyEnum {
    #[serde(rename = "CREATE")]
    Create(SocketInfo<DataId>),
    #[serde(rename = "CONNECT")]
    Connect(DataConnectionIdWrapper),
    #[serde(rename = "DELETE")]
    Delete(DataIdWrapper),
    #[serde(rename = "DISCONNECT")]
    Disconnect(DataConnectionIdWrapper),
    #[serde(rename = "REDIRECT")]
    Redirect(DataConnectionIdWrapper),
    #[serde(rename = "EVENT")]
    Event(DataConnectionEventEnum),
    #[serde(rename = "STATUS")]
    Status(DataConnectionStatus),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "type")]
pub enum ResponseMessageBodyEnum {
    #[serde(rename = "PEER")]
    Peer(PeerResponseMessageBodyEnum),
    #[serde(rename = "DATA")]
    Data(DataResponseMessageBodyEnum),
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub enum Response {
    Success(ResponseMessageBodyEnum),
    Error(String),
}

impl Response {
    pub fn from_str(json: &str) -> Result<Response, error::Error> {
        #[derive(Deserialize)]
        struct ResponseMessageStruct {
            is_success: bool,
            result: Value,
        }
        let value = serde_json::from_str::<ResponseMessageStruct>(json)
            .map_err(|e| error::Error::SerdeError { error: e })?;
        match value.is_success {
            true => {
                let content: ResponseMessageBodyEnum = serde_json::from_value(value.result)
                    .map_err(|e| error::Error::SerdeError { error: e })?;
                Ok(Response::Success(content))
            }
            _ => {
                let content: String = serde_json::from_value(value.result)
                    .map_err(|e| error::Error::SerdeError { error: e })?;
                Ok(Response::Error(content))
            }
        }
    }
}

impl Serialize for Response {
    fn serialize<S>(&self, serializer: S) -> Result<<S as Serializer>::Ok, <S as Serializer>::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("Person", 2)?;
        match self {
            Response::Success(value) => {
                state.serialize_field("is_success", &true)?;
                state.serialize_field("result", &value)?;
            }
            Response::Error(value) => {
                state.serialize_field("is_success", &false)?;
                state.serialize_field("result", &value)?;
            }
        }
        state.end()
    }
}

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
