// skywayプロジェクト全体をDomainモデルとして整理すると
// rust_module以下はApplicationの一部とDomain、Infra層に相当する
// skyway_webrtc_gateway_controller crate(以下SkyWay Crate)をInfra層として利用し、
// ROS側で持つべきDomain知識を定義し、サービスを提供するのが主な目的である

mod application;
mod di;
mod domain;
mod error;
mod ffi;
mod infra;
mod utils;

use std::collections::HashMap;
use std::sync::Arc;

use once_cell::sync::OnceCell;
use shaku::{Component, Interface};
use tokio::sync::Mutex;
use tokio::sync::{mpsc, oneshot};

use crate::domain::entity::{DataConnectionId, MediaConnectionId};
use crate::ffi::global_params::{CallbackFunctions, DataConnectionResponse, Logger, ProgramState};

#[cfg(test)]
use mockall::automock;

// C++側とオブジェクトのやり取りをする回数を最低限にするため、C++側のモジュールで本来所持するべきオブジェクトはOnceCellで保持する
//========== ↓ OnceCell ↓ ==========
// C++側に返すべきコールバックを保持する
static CALLBACK_FUNCTIONS: OnceCell<CallbackFunctions> = OnceCell::new();
// Log出力するために必要なC++側の関数を保持する
static LOGGER_INSTANCE: OnceCell<Logger> = OnceCell::new();
// Programの状態を取得・操作するために必要なC++側の関数を保持する
static PROGRAM_STATE_INSTANCE: OnceCell<ProgramState> = OnceCell::new();
// WebRTC Crate起動時に生成されたSender, Receiverを破棄すると通信できなくなるので、保持し続ける
static CHANNELS: OnceCell<Arc<dyn Channels>> = OnceCell::new();
// Event処理やDisconnect時に利用するため、DataConnection確立時に
// Source Topic とDestination Topicの情報を集めておく
static DATA_CONNECTION_STATE_INSTANCE: OnceCell<
    std::sync::Mutex<HashMap<DataConnectionId, DataConnectionResponse>>,
> = OnceCell::new();

pub(crate) trait Channels: Interface {
    fn sender(&self) -> &mpsc::Sender<(oneshot::Sender<String>, String)>;
    fn receiver(&self) -> &Mutex<mpsc::Receiver<String>>;
}

pub(crate) struct ChannelsImpl {
    sender: mpsc::Sender<(oneshot::Sender<String>, String)>,
    receiver: Mutex<mpsc::Receiver<String>>,
}

impl Channels for ChannelsImpl {
    fn sender(&self) -> &mpsc::Sender<(oneshot::Sender<String>, String)> {
        &self.sender
    }

    fn receiver(&self) -> &Mutex<mpsc::Receiver<String>> {
        &self.receiver
    }
}

#[cfg_attr(test, automock)]
pub(crate) trait GlobalState: Interface {
    fn channels(&self) -> &'static Arc<dyn Channels>;
    fn program_state(&self) -> &'static ProgramState;
    fn store_topic(&self, data_connection_id: DataConnectionId, response: DataConnectionResponse);
    fn find_topic(&self, data_connection_id: &DataConnectionId) -> Option<DataConnectionResponse>;
}

#[derive(Component)]
#[shaku(interface = GlobalState)]
pub(crate) struct GlobalStateImpl {}

impl GlobalState for GlobalStateImpl {
    fn channels(&self) -> &'static Arc<dyn Channels> {
        CHANNELS.get().expect("CHANNELS is not initialized")
    }

    fn program_state(&self) -> &'static ProgramState {
        PROGRAM_STATE_INSTANCE
            .get()
            .expect("PROGRAM_STATE is not initialized")
    }

    fn store_topic(&self, data_connection_id: DataConnectionId, response: DataConnectionResponse) {
        let hash = DATA_CONNECTION_STATE_INSTANCE.get().unwrap();
        hash.lock().unwrap().insert(data_connection_id, response);
    }

    fn find_topic(&self, data_connection_id: &DataConnectionId) -> Option<DataConnectionResponse> {
        let hash = DATA_CONNECTION_STATE_INSTANCE
            .get()
            .unwrap()
            .lock()
            .unwrap();
        let item = hash.get(data_connection_id);
        item.map(|item| item.clone())
    }
}

//========== ↑ OnceCell ↑ ==========

pub(crate) async fn rust_main() {
    DATA_CONNECTION_STATE_INSTANCE.set(std::sync::Mutex::new(HashMap::new()));

    let (sender, receiver) = skyway_webrtc_gateway_caller::run("http://localhost:8000").await;
    // SkyWay Crateにアクセスするためのsender, receiverを保持する
    // Channels objectに入れた上でOnceCellで保持する
    let channels = ChannelsImpl {
        sender,
        receiver: tokio::sync::Mutex::new(receiver),
    };
    let result = CHANNELS.set(Arc::new(channels));
    if result.is_err() {
        Logger::global().error("CHANNELS set error");
        ProgramState::global().shutdown();
    }

    // ROS Serviceからの操作を別スレッドで受け付ける。
    // ROSが終了するまで待機する
    ProgramState::global().wait_for_shutdown();
}
