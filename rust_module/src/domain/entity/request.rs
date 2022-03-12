use serde::{Deserialize, Serialize};

use super::{
    ConnectQuery, CreatePeerParams, DataConnectionIdWrapper, DataIdWrapper, FromStr, PeerInfo,
    RedirectParams, Stringify,
};
use crate::error;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "command")]
pub enum PeerRequest {
    #[serde(rename = "CREATE")]
    Create { params: CreatePeerParams },
    #[serde(rename = "STATUS")]
    Status { params: PeerInfo },
    #[serde(rename = "DELETE")]
    Delete { params: PeerInfo },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(tag = "command")]
pub enum DataRequest {
    #[serde(rename = "CREATE")]
    Create { params: bool },
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
    Peer(PeerRequest),
    #[serde(rename = "DATA")]
    Data(DataRequest),
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
