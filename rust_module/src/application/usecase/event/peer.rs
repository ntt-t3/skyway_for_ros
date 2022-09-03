use std::thread::sleep;
use std::time::Duration;

use shaku::HasComponent;

use super::EventReceiveImpl;
use crate::application::dto::response::PeerResponseDto;
use crate::di::*;
use crate::domain::entity::response::PeerResponse;
use crate::domain::entity::PeerEventEnum;
use crate::error;
use crate::ffi::rust_to_c_bridge::state_objects::ProgramState;

impl EventReceiveImpl {
    pub fn process_peer_event(
        &self,
        response: PeerResponse,
    ) -> Result<PeerResponseDto, error::Error> {
        match response {
            PeerResponse::Event(PeerEventEnum::CLOSE(close)) => {
                std::thread::spawn(|| {
                    sleep(Duration::from_millis(100));
                    let module = CppObjctsModule::builder().build();
                    let state: &dyn ProgramState = module.resolve_ref();
                    state.shutdown();
                });

                Ok(PeerResponseDto::Event(PeerEventEnum::CLOSE(close)))
            }
            PeerResponse::Event(event) => Ok(PeerResponseDto::Event(event)),
            _ => {
                let message = format!("Non-Event object is processed in EventReceiveImpl as Peer");
                self.logger.error(&message);
                unreachable!()
            }
        }
    }
}
