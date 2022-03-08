use serde::{Deserialize, Serialize};

use crate::domain::entity::*;
use crate::error;

// JSONでクライアントから受け取るメッセージ
// JSONとしてなので、キャメルケースではなくスネークケースで受け取る
#[allow(non_camel_case_types)]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "type")]
pub(crate) enum Dto {
    #[serde(rename = "PEER")]
    Peer(PeerServiceParams),
    Data,
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
            Dto::Data => "DATA".to_string(),
            Dto::Media => "MEDIA".to_string(),
            #[cfg(test)]
            _ => "TEST".to_string(),
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
            _ => {
                todo!()
            }
        }
    }
}