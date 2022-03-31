use std::sync::Arc;

use shaku::HasComponent;

use crate::application::dto::request::RequestDto;
use crate::application::usecase::Service;
use crate::di::*;

pub(crate) fn factory(_message: &RequestDto) -> Arc<dyn Service> {
    // TODO
    let module = PeerCreateService::builder().build();
    module.resolve()
}
