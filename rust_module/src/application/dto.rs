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
}
