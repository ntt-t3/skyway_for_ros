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
