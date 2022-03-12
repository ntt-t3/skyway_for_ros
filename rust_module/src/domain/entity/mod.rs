pub(crate) mod request;
pub(crate) mod response;

use crate::error;

// WebRTC Crateの中のこれらのオブジェクトをentityとして再定義
// module内のオブジェクトは、この場所と、crate::errorでError定義を参照するのみで、
// その他の場所からは直接参照させない
pub(crate) use module::prelude::{
    ConnectQuery, CreatePeerParams, DataConnectionEventEnum, DataConnectionId,
    DataConnectionIdWrapper, DataConnectionStatus, DataId, DataIdWrapper, PeerEventEnum, PeerId,
    PeerInfo, PeerStatusMessage, PhantomId, RedirectParams, SerializableId, SerializableSocket,
    SocketInfo, Token,
};

// メッセージを自然にStringに変換できるようにする
pub(crate) trait Stringify {
    fn to_string(&self) -> Result<String, error::Error>;
}

// 自然にStrからメッセージに変換できるようにする
pub(crate) trait FromStr: Sized {
    fn from_str(raw_message: &str) -> Result<Self, error::Error>;
}
