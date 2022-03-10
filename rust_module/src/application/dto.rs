use serde::{Deserialize, Serialize};

use crate::domain::entity::*;
use crate::error;

// JSONでクライアントから受け取るメッセージ
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "type")]
pub(crate) enum Dto {
    #[serde(rename = "PEER")]
    Peer(PeerServiceParams),
    #[serde(rename = "DATA")]
    Data(DataDtoParams),
    Media,
    #[cfg(test)]
    Test,
}

impl Dto {
    pub fn from_str(json: &str) -> Result<Self, error::Error> {
        serde_json::from_str::<Dto>(json).map_err(|e| error::Error::SerdeError { error: e })
    }

    pub fn dto_type(&self) -> String {
        match self {
            Dto::Peer(ref _p) => "PEER".to_string(),
            Dto::Data(ref _d) => "DATA".to_string(),
            Dto::Media => "MEDIA".to_string(),
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
pub(crate) enum DataDtoParams {
    #[serde(rename = "CONNECT")]
    Connect { params: ConnectParams },
}

impl Command for DataDtoParams {
    fn command(&self) -> String {
        match self {
            DataDtoParams::Connect { .. } => "CONNECT".to_string(),
            _ => todo!(),
        }
    }
}
pub(crate) trait Command {
    fn command(&self) -> String;
}

impl Command for PeerServiceParams {
    fn command(&self) -> String {
        match self {
            PeerServiceParams::Create { params: ref _p } => "CREATE".to_string(),
            PeerServiceParams::Delete { params: ref _p } => "DELETE".to_string(),
            PeerServiceParams::Status { params: ref _p } => "STATUS".to_string(),
        }
    }
}

impl Command for Dto {
    fn command(&self) -> String {
        match self {
            Dto::Peer(ref peer) => peer.command(),
            Dto::Data(ref data) => data.command(),
            _ => {
                todo!()
            }
        }
    }
}
