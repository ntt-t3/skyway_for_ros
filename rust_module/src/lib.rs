// skywayプロジェクト全体をDomainモデルとして整理すると
// rust_module以下はApplicationの一部とDomain、Infra層に相当する
// skyway_webrtc_gateway_controller crate(以下SkyWay Crate)をInfra層として利用し、
// ROS側で持つべきDomain知識を定義し、サービスを提供するのが主な目的である

mod application;
mod domain;
mod error;
mod ffi;
mod infra;
mod utils;

use std::collections::HashMap;
use std::sync::Mutex;

use once_cell::sync::OnceCell;

use crate::application::dto::response::CallResponseDto;
use crate::domain::entity::{DataConnectionId, MediaConnectionId};
use crate::domain::repository::Repository;
use crate::ffi::*;

// C++側とオブジェクトのやり取りをする回数を最低限にするため、C++側のモジュールで本来所持するべきオブジェクトはOnceCellで保持する
//========== ↓ OnceCell ↓ ==========
// C++側に返すべきコールバックを保持する
static CALLBACK_FUNCTIONS: OnceCell<CallbackFunctions> = OnceCell::new();
// Log出力するために必要なC++側の関数を保持する
static LOGGER_INSTANCE: OnceCell<Logger> = OnceCell::new();
// Programの状態を取得・操作するために必要なC++側の関数を保持する
static PROGRAM_STATE_INSTANCE: OnceCell<ProgramState> = OnceCell::new();
// RepositoryとしてWebRTC Crateを利用しているが、生成されたSender, Receiverを破棄すると通信できなくなるので、保持し続ける
static REPOSITORY_INSTANCE: OnceCell<Box<dyn Repository>> = OnceCell::new();
// OPENイベント時に返すため、MediaConnection確立時に情報を集めておく
static MEDIA_CONNECTION_STATE_INSTANCE: OnceCell<
    Mutex<HashMap<MediaConnectionId, CallResponseDto>>,
> = OnceCell::new();
// OPENイベント時に返すため、DataConnection確立時に情報を集めておく
static DATA_CONNECTION_STATE_INSTANCE: OnceCell<
    Mutex<HashMap<DataConnectionId, DataConnectionResponse>>,
> = OnceCell::new();

//========== ↑ OnceCell ↑ ==========
