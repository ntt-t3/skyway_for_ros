use serde::ser::SerializeStruct;
use serde::{Deserialize, Serialize, Serializer};

use crate::domain::entity::{
    DataConnectionEventEnum, DataConnectionId, DataConnectionIdWrapper, DataConnectionStatus,
    DataId, DataIdWrapper, DataResponseMessageBodyEnum, PeerEventEnum, PeerInfo,
    PeerResponseMessageBodyEnum, PeerStatusMessage, SocketInfo,
};
use crate::error;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "command")]
pub enum PeerResponseDto {
    #[serde(rename = "CREATE")]
    Create(PeerInfo),
    #[serde(rename = "STATUS")]
    Status(PeerStatusMessage),
    #[serde(rename = "DELETE")]
    Delete(PeerInfo),
    #[serde(rename = "EVENT")]
    Event(PeerEventEnum),
}

impl PeerResponseDto {
    pub fn from_entity(entity: PeerResponseMessageBodyEnum) -> Self {
        match entity {
            PeerResponseMessageBodyEnum::Create(item) => PeerResponseDto::Create(item),
            PeerResponseMessageBodyEnum::Delete(item) => PeerResponseDto::Delete(item),
            PeerResponseMessageBodyEnum::Status(item) => PeerResponseDto::Status(item),
            PeerResponseMessageBodyEnum::Event(item) => PeerResponseDto::Event(item),
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
pub enum DataResponseDto {
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

impl DataResponseDto {
    pub fn from_entity(entity: DataResponseMessageBodyEnum) -> Self {
        match entity {
            DataResponseMessageBodyEnum::Create(item) => DataResponseDto::Create(item),
            DataResponseMessageBodyEnum::Connect(item) => {
                let data = DataConnectionResponse {
                    data_connection_id: item.data_connection_id,
                    source_topic_name: "".to_string(),
                    source_ip: "".to_string(),
                    source_port: 0,
                    destination_topic_name: "".to_string(),
                };
                DataResponseDto::Connect(data)
            }
            DataResponseMessageBodyEnum::Delete(item) => DataResponseDto::Delete(item),
            DataResponseMessageBodyEnum::Disconnect(item) => DataResponseDto::Disconnect(item),
            DataResponseMessageBodyEnum::Redirect(item) => {
                let data = DataConnectionResponse {
                    data_connection_id: item.data_connection_id,
                    source_topic_name: "".to_string(),
                    source_ip: "".to_string(),
                    source_port: 0,
                    destination_topic_name: "".to_string(),
                };
                DataResponseDto::Redirect(data)
            }
            DataResponseMessageBodyEnum::Event(item) => DataResponseDto::Event(item),
            DataResponseMessageBodyEnum::Status(item) => DataResponseDto::Status(item),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "type")]
pub enum ResponseDto {
    #[serde(rename = "PEER")]
    Peer(PeerResponseDto),
    #[serde(rename = "DATA")]
    Data(DataResponseDto),
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub enum ResponseDtoResult {
    Success(ResponseDto),
    Error(String),
}

impl ResponseDtoResult {
    pub fn from_str(json: &str) -> Result<ResponseDtoResult, error::Error> {
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
                let content: ResponseDto = serde_json::from_value(value.result)
                    .map_err(|e| error::Error::SerdeError { error: e })?;
                Ok(ResponseDtoResult::Success(content))
            }
            _ => {
                let content: String = serde_json::from_value(value.result)
                    .map_err(|e| error::Error::SerdeError { error: e })?;
                Ok(ResponseDtoResult::Error(content))
            }
        }
    }

    pub fn to_string(&self) -> Result<String, error::Error> {
        serde_json::to_string(self).map_err(|e| error::Error::SerdeError { error: e })
    }
}

impl Serialize for ResponseDtoResult {
    fn serialize<S>(&self, serializer: S) -> Result<<S as Serializer>::Ok, <S as Serializer>::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("Person", 2)?;
        match self {
            ResponseDtoResult::Success(value) => {
                state.serialize_field("is_success", &true)?;
                state.serialize_field("result", &value)?;
            }
            ResponseDtoResult::Error(value) => {
                state.serialize_field("is_success", &false)?;
                state.serialize_field("result", &value)?;
            }
        }
        state.end()
    }
}
