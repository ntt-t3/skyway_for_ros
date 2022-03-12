pub(crate) mod request;
pub(crate) mod response;

use serde::ser::SerializeStruct;
use serde::{Deserialize, Serialize, Serializer};

use crate::domain::entity::*;
use crate::error;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub(crate) struct RedirectDtoParams {
    pub data_connection_id: DataConnectionId,
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
pub struct DataConnectionResponse {
    pub data_connection_id: DataConnectionId,
    pub source_topic_name: String,
    pub source_ip: String,
    pub source_port: u16,
    pub destination_topic_name: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "command")]
pub enum DataDtoResponseMessageBodyEnum {
    #[serde(rename = "CREATE")]
    Create(SocketInfo<DataId>),
    #[serde(rename = "CONNECT")]
    Connect(DataConnectionResponse),
    #[serde(rename = "DELETE")]
    Delete(DataIdWrapper),
    #[serde(rename = "DISCONNECT")]
    Disconnect(DataConnectionIdWrapper),
    #[serde(rename = "REDIRECT")]
    Redirect(DataConnectionResponse),
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
                let data = DataConnectionResponse {
                    data_connection_id: item.data_connection_id,
                    source_topic_name: "".to_string(),
                    source_ip: "".to_string(),
                    source_port: 0,
                    destination_topic_name: "".to_string(),
                };
                DataDtoResponseMessageBodyEnum::Connect(data)
            }
            DataResponseMessageBodyEnum::Delete(item) => {
                DataDtoResponseMessageBodyEnum::Delete(item)
            }
            DataResponseMessageBodyEnum::Disconnect(item) => {
                DataDtoResponseMessageBodyEnum::Disconnect(item)
            }
            DataResponseMessageBodyEnum::Redirect(item) => {
                let data = DataConnectionResponse {
                    data_connection_id: item.data_connection_id,
                    source_topic_name: "".to_string(),
                    source_ip: "".to_string(),
                    source_port: 0,
                    destination_topic_name: "".to_string(),
                };
                DataDtoResponseMessageBodyEnum::Redirect(data)
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

#[derive(Debug, Clone, PartialEq, Deserialize)]
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

impl Serialize for ResponseDto {
    fn serialize<S>(&self, serializer: S) -> Result<<S as Serializer>::Ok, <S as Serializer>::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("Person", 2)?;
        match self {
            ResponseDto::Success(value) => {
                state.serialize_field("is_success", &true)?;
                state.serialize_field("result", &value)?;
            }
            ResponseDto::Error(value) => {
                state.serialize_field("is_success", &false)?;
                state.serialize_field("result", &value)?;
            }
        }
        state.end()
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
