use shaku::module;

use crate::application::factory::FactoryImpl;
use crate::application::usecase::data::connect::Connect;
use crate::application::usecase::data::redirect::Redirect;
use crate::application::usecase::event;
use crate::application::usecase::media::answer::AnswerService;
use crate::application::usecase::General;
use crate::infra::RepositoryImpl;
use crate::utils::CallbackCallerImpl;
use crate::GlobalStateImpl;

module! {
    pub(crate) GeneralFactory {
        components = [FactoryImpl],
        providers = []
    }
}

module! {
    pub(crate) RepositoryModule {
        components = [RepositoryImpl, GlobalStateImpl],
        providers = []
    }
}

module! {
    pub(crate) GeneralService {
        components = [General, RepositoryImpl, GlobalStateImpl],
        providers = []
    }
}

module! {
    pub(crate) DataConnectService {
        components = [Connect, GlobalStateImpl, RepositoryImpl, FactoryImpl, CallbackCallerImpl],
        providers = []
    }
}

module! {
    pub(crate) DataRedirectService {
        components = [Redirect, GlobalStateImpl, RepositoryImpl, FactoryImpl, CallbackCallerImpl],
        providers = []
    }
}

module! {
    pub(crate) MediaAnswerService {
        components = [AnswerService, GlobalStateImpl, RepositoryImpl, FactoryImpl, CallbackCallerImpl],
        providers = []
    }
}

module! {
    pub(crate) EventReceiveService {
        components = [event::EventReceiveImpl, GlobalStateImpl, RepositoryImpl],
        providers = []
    }
}
