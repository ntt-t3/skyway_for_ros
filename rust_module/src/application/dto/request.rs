use serde::{Deserialize, Serialize};

use crate::application::dto::response::MediaInfo;
use crate::application::dto::Command;
pub(crate) use crate::domain::entity::request::PeerRequest as PeerRequestDto;
use crate::domain::entity::request::{AnswerParameters, IsVideo};
use crate::domain::entity::{
    CallQuery, DataConnectionId, DataConnectionIdWrapper, DataIdWrapper, MediaConnectionId,
    MediaIdWrapper, PeerId, Token,
};
use crate::error;

//========== Peer ==========
impl Command for PeerRequestDto {
    fn command(&self) -> String {
        match self {
            PeerRequestDto::Create { params: ref _p } => "CREATE".to_string(),
            PeerRequestDto::Delete { params: ref _p } => "DELETE".to_string(),
            PeerRequestDto::Status { params: ref _p } => "STATUS".to_string(),
        }
    }
}

//========== Media ==========

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub(crate) struct CallResponseDto {
    pub video: MediaInfo,
    pub audio: MediaInfo,
    pub media_connection_id: MediaConnectionId,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "command")]
pub(crate) enum MediaRequestDto {
    #[serde(rename = "CONTENT_CREATE")]
    ContentCreate { params: IsVideo },
    #[serde(rename = "CONTENT_DELETE")]
    ContentDelete { params: MediaIdWrapper },
    #[serde(rename = "RTCP_CREATE")]
    RtcpCreate { params: Option<()> },
    #[serde(rename = "CALL")]
    Call { params: CallQuery },
    #[serde(rename = "ANSWER")]
    Answer { params: AnswerParameters },
}

//========== Data ==========
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub(crate) struct ConnectDtoParams {
    pub peer_id: PeerId,
    pub token: Token,
    pub target_id: PeerId,
    pub destination_topic: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub(crate) struct RedirectDtoParams {
    pub data_connection_id: DataConnectionId,
    pub destination_topic: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "command")]
pub(crate) enum DataRequestDto {
    #[serde(rename = "CREATE")]
    Create,
    #[serde(rename = "DELETE")]
    Delete { params: DataIdWrapper },
    #[serde(rename = "CONNECT")]
    Connect { params: ConnectDtoParams },
    #[serde(rename = "REDIRECT")]
    Redirect { params: RedirectDtoParams },
    #[serde(rename = "DISCONNECT")]
    Disconnect { params: DataConnectionIdWrapper },
}

impl Command for DataRequestDto {
    fn command(&self) -> String {
        match self {
            DataRequestDto::Create => "CREATE".to_string(),
            DataRequestDto::Delete { .. } => "DELETE".to_string(),
            DataRequestDto::Connect { .. } => "CONNECT".to_string(),
            DataRequestDto::Redirect { .. } => "REDIRECT".to_string(),
            DataRequestDto::Disconnect { .. } => "DISCONNECT".to_string(),
        }
    }
}

//========== General ==========
// JSONでクライアントから受け取るメッセージ
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "type")]
pub(crate) enum RequestDto {
    #[serde(rename = "PEER")]
    Peer(PeerRequestDto),
    #[serde(rename = "DATA")]
    Data(DataRequestDto),
    #[serde(rename = "MEDIA")]
    Media(MediaRequestDto),
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
            RequestDto::Media(ref _m) => "MEDIA".to_string(),
            #[cfg(test)]
            _ => "TEST".to_string(),
        }
    }

    pub fn to_string(&self) -> Result<String, error::Error> {
        serde_json::to_string(self).map_err(|e| error::Error::SerdeError { error: e })
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
