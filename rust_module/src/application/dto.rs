use serde::{Deserialize, Serialize};

use crate::domain::entity::*;
use crate::error;

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "command")]
pub(crate) enum DataRequestDtoParams {
    #[serde(rename = "CREATE")]
    Create,
    #[serde(rename = "DELETE")]
    Delete { params: DataIdWrapper },
    #[serde(rename = "CONNECT")]
    Connect { params: ConnectParams },
    #[serde(rename = "REDIRECT")]
    Redirect { params: RedirectParams },
    #[serde(rename = "DISCONNECT")]
    Disconnect { params: DataConnectionIdWrapper },
}

impl Command for DataRequestDtoParams {
    fn command(&self) -> String {
        match self {
            DataRequestDtoParams::Create => "CREATE".to_string(),
            DataRequestDtoParams::Delete { .. } => "DELETE".to_string(),
            DataRequestDtoParams::Connect { .. } => "CONNECT".to_string(),
            DataRequestDtoParams::Redirect { .. } => "REDIRECT".to_string(),
            DataRequestDtoParams::Disconnect { .. } => "DISCONNECT".to_string(),
        }
    }
}
// JSONでクライアントから受け取るメッセージ
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "type")]
pub(crate) enum RequestDto {
    #[serde(rename = "PEER")]
    Peer(PeerRequestParams),
    #[serde(rename = "DATA")]
    Data(DataRequestDtoParams),
    Media,
    #[cfg(test)]
    Test,
}

impl RequestDto {
    pub fn from_str(json: &str) -> Result<Self, error::Error> {
        serde_json::from_str::<RequestDto>(json).map_err(|e| error::Error::SerdeError { error: e })
    }

    pub fn dto_type(&self) -> String {
        match self {
            RequestDto::Peer(ref _p) => "PEER".to_string(),
            RequestDto::Data(ref _d) => "DATA".to_string(),
            RequestDto::Media => "MEDIA".to_string(),
            #[cfg(test)]
            _ => "TEST".to_string(),
        }
    }

    pub fn to_string(&self) -> Result<String, error::Error> {
        serde_json::to_string(self).map_err(|e| error::Error::SerdeError { error: e })
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub(crate) struct ConnectParams {
    pub peer_id: PeerId,
    pub token: Token,
    pub target_id: PeerId,
    pub destination_topic: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "command")]
pub enum PeerDtoResponseMessageBodyEnum {
    #[serde(rename = "CREATE")]
    Create(PeerInfo),
    #[serde(rename = "STATUS")]
    Status(PeerStatusMessage),
    #[serde(rename = "DELETE")]
    Delete(PeerInfo),
    #[serde(rename = "EVENT")]
    Event(PeerEventEnum),
}

impl PeerDtoResponseMessageBodyEnum {
    pub fn from_entity(entity: PeerResponseMessageBodyEnum) -> Self {
        match entity {
            PeerResponseMessageBodyEnum::Create(item) => {
                PeerDtoResponseMessageBodyEnum::Create(item)
            }
            PeerResponseMessageBodyEnum::Delete(item) => {
                PeerDtoResponseMessageBodyEnum::Delete(item)
            }
            PeerResponseMessageBodyEnum::Status(item) => {
                PeerDtoResponseMessageBodyEnum::Status(item)
            }
            PeerResponseMessageBodyEnum::Event(item) => PeerDtoResponseMessageBodyEnum::Event(item),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "command")]
pub enum DataDtoResponseMessageBodyEnum {
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

impl DataDtoResponseMessageBodyEnum {
    pub fn from_entity(entity: DataResponseMessageBodyEnum) -> Self {
        match entity {
            DataResponseMessageBodyEnum::Create(item) => {
                DataDtoResponseMessageBodyEnum::Create(item)
            }
            DataResponseMessageBodyEnum::Connect(item) => {
                DataDtoResponseMessageBodyEnum::Connect(item)
            }
            DataResponseMessageBodyEnum::Delete(item) => {
                DataDtoResponseMessageBodyEnum::Delete(item)
            }
            DataResponseMessageBodyEnum::Disconnect(item) => {
                DataDtoResponseMessageBodyEnum::Disconnect(item)
            }
            DataResponseMessageBodyEnum::Redirect(item) => {
                DataDtoResponseMessageBodyEnum::Redirect(item)
            }
            DataResponseMessageBodyEnum::Event(item) => DataDtoResponseMessageBodyEnum::Event(item),
            DataResponseMessageBodyEnum::Status(item) => {
                DataDtoResponseMessageBodyEnum::Status(item)
            }
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "type")]
pub enum ResponseDtoMessageBodyEnum {
    #[serde(rename = "PEER")]
    Peer(PeerDtoResponseMessageBodyEnum),
    #[serde(rename = "DATA")]
    Data(DataDtoResponseMessageBodyEnum),
}

#[derive(Serialize, Debug, Clone, PartialEq, Deserialize)]
pub enum ResponseDto {
    Success(ResponseDtoMessageBodyEnum),
    Error(String),
}

impl ResponseDto {
    pub fn from_str(json: &str) -> Result<ResponseDto, error::Error> {
        #[allow(dead_code)]
        #[derive(Deserialize)]
        struct ResponseMessageStruct {
            is_success: bool,
            result: serde_json::Value,
        }
        let value = serde_json::from_str::<ResponseMessageStruct>(json)
            .map_err(|e| error::Error::SerdeError { error: e })?;
        match value.is_success {
            true => {
                let content: ResponseDtoMessageBodyEnum = serde_json::from_value(value.result)
                    .map_err(|e| error::Error::SerdeError { error: e })?;
                Ok(ResponseDto::Success(content))
            }
            _ => {
                let content: String = serde_json::from_value(value.result)
                    .map_err(|e| error::Error::SerdeError { error: e })?;
                Ok(ResponseDto::Error(content))
            }
        }
    }

    pub fn to_string(&self) -> Result<String, error::Error> {
        serde_json::to_string(self).map_err(|e| error::Error::SerdeError { error: e })
    }
}

pub(crate) trait Command {
    fn command(&self) -> String;
}

impl Command for PeerRequestParams {
    fn command(&self) -> String {
        match self {
            PeerRequestParams::Create { params: ref _p } => "CREATE".to_string(),
            PeerRequestParams::Delete { params: ref _p } => "DELETE".to_string(),
            PeerRequestParams::Status { params: ref _p } => "STATUS".to_string(),
        }
    }
}

impl Command for RequestDto {
    fn command(&self) -> String {
        match self {
            RequestDto::Peer(ref peer) => peer.command(),
            RequestDto::Data(ref data) => data.command(),
            _ => {
                todo!()
            }
        }
    }
}
