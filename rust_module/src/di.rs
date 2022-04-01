use shaku::module;

use crate::application::usecase::event::EventReceiveImpl;
use crate::application::usecase::peer::create::Create;
use crate::infra::RepositoryImpl;
use crate::GlobalStateImpl;

module! {
    pub(crate) RepositoryModule {
        components = [RepositoryImpl, GlobalStateImpl],
        providers = []
    }
}

module! {
    pub(crate) PeerCreateService {
        components = [Create, GlobalStateImpl, RepositoryImpl],
        providers = []
    }
}

module! {
    pub(crate) EventService {
        components = [EventReceiveImpl, GlobalStateImpl, RepositoryImpl],
        providers = []
    }
}
