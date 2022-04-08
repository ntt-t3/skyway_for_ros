use shaku::module;

use crate::application::factory::FactoryImpl;
use crate::application::usecase::data::connect::Connect;
use crate::application::usecase::data::redirect::Redirect;
use crate::application::usecase::event;
use crate::application::usecase::media::answer::AnswerService;
use crate::application::usecase::media::call::Call;
use crate::application::usecase::peer::create::Create;
use crate::application::usecase::General;
use crate::infra::RepositoryImpl;
use crate::utils::CallbackCallerImpl;
use crate::{CallbackFunctionsImpl, GlobalStateImpl, LoggerImpl, ProgramStateImpl};

module! {
    pub(crate) CppObjctsModule {
        components = [ProgramStateImpl, CallbackFunctionsImpl, LoggerImpl, GlobalStateImpl],
        providers = []
    }
}

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
    pub(crate) PeerCreateService {
        components = [Create, GlobalStateImpl, RepositoryImpl, FactoryImpl, CallbackCallerImpl],
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
    pub(crate) MediaCallService {
        components = [Call, GlobalStateImpl, RepositoryImpl, FactoryImpl, CallbackCallerImpl],
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
        components = [event::EventReceiveImpl, CallbackCallerImpl, GlobalStateImpl, RepositoryImpl, LoggerImpl],
        providers = []
    }
}
