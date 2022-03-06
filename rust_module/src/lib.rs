// skywayプロジェクト全体をDomainモデルとして整理すると
// rust_module以下はApplicationの一部とDomain、Infra層に相当する
// skyway_webrtc_gateway_controller crate(以下SkyWay Crate)をInfra層として利用し、
// ROS側で持つべきDomain知識を定義し、サービスを提供するのが主な目的である

pub(crate) mod c_module;
mod domain;
mod error;
mod infra;

use c_module::*;

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

    //ProgramState::global().wait_for_shutdown();
    return true;
}
