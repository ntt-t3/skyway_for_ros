// skywayプロジェクト全体をDomainモデルとして整理すると
// rust_module以下はApplicationの一部とDomain、Infra層に相当する
// skyway_webrtc_gateway_controller crate(以下SkyWay Crate)をInfra層として利用し、
// ROS側で持つべきDomain知識を定義し、サービスを提供するのが主な目的である

pub(crate) mod c_module;
mod domain;
mod error;
mod infra;

use std::sync::Mutex;

use once_cell::sync::OnceCell;

use crate::c_module::*;
use crate::domain::repository::Repository;
use crate::infra::RepositoryImpl;

static REPOSITORY_INSTANCE: OnceCell<Mutex<Box<dyn Repository + Send + Sync>>> = OnceCell::new();

#[no_mangle]
pub extern "C" fn run() -> bool {
    if !Logger::is_allocated() {
        return false;
    }
    if !ProgramState::is_allocated() {
        Logger::global().error(
            "ProgramState object is not allocated. Please call the register_program_state function",
        );
        return false;
    }

    // SkyWay Crateを開始する
    let rt = tokio::runtime::Runtime::new().unwrap();
    rt.block_on(async {
        let (sender, receiver) = module::run("http://localhost:8000").await;
        // SkyWay Crateにアクセスするためのsender, receiverはRepositoryの中で保持する
        // Repositoryはonce_cellでglobalで確保される
        let repository = RepositoryImpl::new(sender, receiver);
        if REPOSITORY_INSTANCE
            .set(Mutex::new(Box::new(repository)))
            .is_err()
        {
            return;
        }
        ProgramState::global().wait_for_shutdown();
    });
    return true;
}
