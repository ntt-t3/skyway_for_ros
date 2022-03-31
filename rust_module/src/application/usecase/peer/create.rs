use std::sync::Arc;

use shaku::Component;

use crate::application::usecase::Service;
use crate::domain::repository::Repository;
use crate::GlobalState;

#[derive(Component)]
#[shaku(interface = Service)]
pub(crate) struct Create {
    #[shaku(inject)]
    repository: Arc<dyn Repository>,
    #[shaku(inject)]
    state: Arc<dyn GlobalState>,
}

impl Service for Create {}
