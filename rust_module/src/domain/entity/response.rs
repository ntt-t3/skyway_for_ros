use serde::ser::SerializeStruct;
use serde::{Deserialize, Serialize, Serializer};
use serde_json::Value;

use crate::domain::entity::{
    AnswerResult, DataConnectionEventEnum, DataConnectionIdWrapper, DataConnectionStatus, DataId,
    DataIdWrapper, FromStr, MediaConnectionEventEnum, MediaConnectionIdWrapper,
    MediaConnectionStatus, MediaId, MediaIdWrapper, PeerEventEnum, PeerInfo, PeerStatusMessage,
    RtcpId, RtcpIdWrapper, SocketInfo, Stringify,
};
use crate::error;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "command")]
pub enum PeerResponse {
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
pub enum DataResponse {
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
#[serde(tag = "command")]
pub(crate) enum MediaResponse {
    #[serde(rename = "CONTENT_CREATE")]
    ContentCreate(SocketInfo<MediaId>),
    #[serde(rename = "CONTENT_DELETE")]
    ContentDelete(MediaIdWrapper),
    #[serde(rename = "RTCP_CREATE")]
    RtcpCreate(SocketInfo<RtcpId>),
    #[serde(rename = "RTCP_DELETE")]
    RtcpDelete(RtcpIdWrapper),
    #[serde(rename = "CALL")]
    Call(MediaConnectionIdWrapper),
    #[serde(rename = "ANSWER")]
    Answer(AnswerResult),
    #[serde(rename = "EVENT")]
    Event(MediaConnectionEventEnum),
    #[serde(rename = "STATUS")]
    Status(MediaConnectionStatus),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "type")]
pub(crate) enum Response {
    #[serde(rename = "PEER")]
    Peer(PeerResponse),
    #[serde(rename = "MEDIA")]
    Media(MediaResponse),
    #[serde(rename = "DATA")]
    Data(DataResponse),
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub(crate) enum ResponseResult {
    Success(Response),
    Error(String),
}

impl ResponseResult {
    pub fn from_str(json: &str) -> Result<ResponseResult, error::Error> {
        #[derive(Deserialize)]
        struct ResponseMessageStruct {
            is_success: bool,
            result: Value,
        }
        let value = serde_json::from_str::<ResponseMessageStruct>(json)
            .map_err(|e| error::Error::SerdeError { error: e })?;
        match value.is_success {
            true => {
                let content: Response = serde_json::from_value(value.result)
                    .map_err(|e| error::Error::SerdeError { error: e })?;
                Ok(ResponseResult::Success(content))
            }
            _ => {
                let content: String = serde_json::from_value(value.result)
                    .map_err(|e| error::Error::SerdeError { error: e })?;
                Ok(ResponseResult::Error(content))
            }
        }
    }
}

impl Serialize for ResponseResult {
    fn serialize<S>(&self, serializer: S) -> Result<<S as Serializer>::Ok, <S as Serializer>::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("Person", 2)?;
        match self {
            ResponseResult::Success(value) => {
                state.serialize_field("is_success", &true)?;
                state.serialize_field("result", &value)?;
            }
            ResponseResult::Error(value) => {
                state.serialize_field("is_success", &false)?;
                state.serialize_field("result", &value)?;
            }
        }
        state.end()
    }
}

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
