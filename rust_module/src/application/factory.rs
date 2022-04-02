use std::sync::Arc;

use async_trait::async_trait;

use shaku::{Component, HasComponent, Interface};

use crate::application::dto::request::{DataRequestDto, RequestDto};
use crate::application::usecase::Service;
use crate::di::*;

#[cfg(test)]
use mockall::automock;

#[async_trait]
#[cfg_attr(test, automock)]
pub(crate) trait Factory: Interface {
    fn create_service(&self, params: &RequestDto) -> Arc<dyn Service>;
}

#[derive(Component)]
#[shaku(interface = Factory)]
pub(crate) struct FactoryImpl {}

impl Factory for FactoryImpl {
    fn create_service(&self, request: &RequestDto) -> Arc<dyn Service> {
        match request {
            RequestDto::Data(DataRequestDto::Create) => {
                let module = DataCreateService::builder().build();
                module.resolve()
            }
            _ => {
                let module = PeerCreateService::builder().build();
                module.resolve()
            }
        }
    }
}
