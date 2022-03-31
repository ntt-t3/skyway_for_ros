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

use std::sync::Arc;

use once_cell::sync::OnceCell;
use shaku::{Component, Interface};
use tokio::sync::Mutex;
use tokio::sync::{mpsc, oneshot};

use crate::domain::entity::MediaConnectionId;
use crate::ffi::global_params::{CallbackFunctions, Logger, ProgramState};

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
}

//========== ↑ OnceCell ↑ ==========
