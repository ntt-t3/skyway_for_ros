pub(crate) mod data;
pub(crate) mod event;
pub(crate) mod peer;

use std::sync::Arc;

use async_trait::async_trait;
use shaku::{Component, Interface};

use crate::application::dto::request::{DataRequestDto, MediaRequestDto, RequestDto};
use crate::application::dto::response::{
    DataResponseDto, MediaResponseDto, PeerResponseDto, ResponseDto, ResponseDtoResult,
};
use crate::domain::entity::request::{DataRequest, MediaRequest, Request};
use crate::domain::entity::response::{
    DataResponse, MediaResponse, PeerResponse, Response, ResponseResult,
};
use crate::domain::repository::Repository;
use crate::error;

#[cfg(test)]
use mockall::automock;

#[cfg_attr(test, automock)]
#[async_trait]
pub(crate) trait Service: Interface {
    async fn execute(&self, request: RequestDto) -> Result<ResponseDtoResult, error::Error>;
}

#[derive(Component)]
#[shaku(interface = Service)]
pub(crate) struct General {
    #[shaku(inject)]
    repository: Arc<dyn Repository>,
}

#[async_trait]
impl Service for General {
    async fn execute(&self, dto: RequestDto) -> Result<ResponseDtoResult, error::Error> {
        let request = dto_to_request(dto)?;
        let result = self.repository.register(request).await?;
        result_to_dto(result)
    }
}

fn dto_to_request(dto: RequestDto) -> Result<Request, error::Error> {
    match dto {
        RequestDto::Peer(parameter) => Ok(Request::Peer(parameter)),
        RequestDto::Data(DataRequestDto::Create) => {
            Ok(Request::Data(DataRequest::Create { params: true }))
        }
        RequestDto::Data(DataRequestDto::Delete { params }) => {
            Ok(Request::Data(DataRequest::Delete { params }))
        }
        RequestDto::Data(DataRequestDto::Disconnect { params }) => {
            Ok(Request::Data(DataRequest::Disconnect { params }))
        }
        RequestDto::Media(MediaRequestDto::ContentCreate { params }) => {
            Ok(Request::Media(MediaRequest::ContentCreate { params }))
        }
        RequestDto::Media(MediaRequestDto::ContentDelete { params }) => {
            Ok(Request::Media(MediaRequest::ContentDelete { params }))
        }
        RequestDto::Media(MediaRequestDto::RtcpCreate { params }) => {
            Ok(Request::Media(MediaRequest::RtcpCreate { params }))
        }
        RequestDto::Media(MediaRequestDto::RtcpDelete { params }) => {
            Ok(Request::Media(MediaRequest::RtcpDelete { params }))
        }
        RequestDto::Media(MediaRequestDto::Disconnect { params }) => {
            Ok(Request::Media(MediaRequest::Disconnect { params }))
        }
        _ => Err(error::Error::create_local_error(
            "invalid parameter for GenealService",
        )),
    }
}

fn result_to_dto(response: ResponseResult) -> Result<ResponseDtoResult, error::Error> {
    match response {
        ResponseResult::Success(Response::Peer(PeerResponse::Create(params))) => Ok(
            ResponseDtoResult::Success(ResponseDto::Peer(PeerResponseDto::Create(params))),
        ),
        ResponseResult::Success(Response::Peer(PeerResponse::Delete(params))) => Ok(
            ResponseDtoResult::Success(ResponseDto::Peer(PeerResponseDto::Delete(params))),
        ),
        ResponseResult::Success(Response::Peer(PeerResponse::Status(params))) => Ok(
            ResponseDtoResult::Success(ResponseDto::Peer(PeerResponseDto::Status(params))),
        ),
        ResponseResult::Success(Response::Data(DataResponse::Create(params))) => Ok(
            ResponseDtoResult::Success(ResponseDto::Data(DataResponseDto::Create(params))),
        ),
        ResponseResult::Success(Response::Data(DataResponse::Delete(params))) => Ok(
            ResponseDtoResult::Success(ResponseDto::Data(DataResponseDto::Delete(params))),
        ),
        ResponseResult::Success(Response::Data(DataResponse::Disconnect(params))) => Ok(
            ResponseDtoResult::Success(ResponseDto::Data(DataResponseDto::Disconnect(params))),
        ),
        ResponseResult::Success(Response::Media(MediaResponse::ContentCreate(params))) => Ok(
            ResponseDtoResult::Success(ResponseDto::Media(MediaResponseDto::ContentCreate(params))),
        ),
        ResponseResult::Success(Response::Media(MediaResponse::ContentDelete(params))) => Ok(
            ResponseDtoResult::Success(ResponseDto::Media(MediaResponseDto::ContentDelete(params))),
        ),
        ResponseResult::Success(Response::Media(MediaResponse::RtcpCreate(params))) => Ok(
            ResponseDtoResult::Success(ResponseDto::Media(MediaResponseDto::RtcpCreate(params))),
        ),
        ResponseResult::Success(Response::Media(MediaResponse::RtcpDelete(params))) => Ok(
            ResponseDtoResult::Success(ResponseDto::Media(MediaResponseDto::RtcpDelete(params))),
        ),
        ResponseResult::Success(Response::Media(MediaResponse::Disconnect(params))) => Ok(
            ResponseDtoResult::Success(ResponseDto::Media(MediaResponseDto::Disconnect(params))),
        ),
        _ => Err(error::Error::create_local_error(
            "invalid response for GenealService",
        )),
    }
}

#[cfg(test)]
mod create_peer_test {
    use shaku::HasComponent;

    use super::*;
    use crate::application::dto::request::RequestDto;
    use crate::application::dto::response::ResponseDtoResult;
    use crate::di::GeneralService;
    use crate::domain::entity::response::ResponseResult;
    use crate::domain::repository::MockRepository;

    #[tokio::test]
    async fn success() {
        // CreatePeerに成功したメッセージが得られるはずである
        let expected = {
            let message = r#"{
                    "is_success":true,
                    "result":{
                        "type":"PEER",
                        "command":"CREATE",
                        "peer_id":"peer_id",
                        "token":"pt-06cf1d26-0ef0-4b03-aca6-933027d434c2"
                    }
                }"#;
            ResponseDtoResult::from_str(message).unwrap()
        };

        // CreatePeerのパラメータ生成
        let dto = {
            let message = r#"{
                "type": "PEER",
                "command": "CREATE",
                "params": {
                    "key": "API_KEY",
                    "domain": "localhost",
                    "peer_id": "peer_id",
                    "turn": true
                }
            }"#;
            RequestDto::from_str(message).unwrap()
        };

        // repositoryのMockを生成
        // 呼び出しに成功するケース
        let mut repository = MockRepository::new();
        repository.expect_register().times(1).returning(|_| {
            let message = r#"{
                    "is_success":true,
                    "result":{
                        "type":"PEER",
                        "command":"CREATE",
                        "peer_id":"peer_id",
                        "token":"pt-06cf1d26-0ef0-4b03-aca6-933027d434c2"
                    }
                }"#;
            ResponseResult::from_str(message)
        });

        // サービスの生成
        let module = GeneralService::builder()
            .with_component_override::<dyn Repository>(Box::new(repository))
            .build();
        let service: &dyn Service = module.resolve_ref();

        // 実行
        let result = service.execute(dto).await;
        assert_eq!(result.unwrap(), expected);
    }

    #[tokio::test]
    async fn fail() {
        // APIがエラーを返してくるケース

        // CreatePeerのパラメータ生成
        let dto = {
            let message = r#"{
                "type": "PEER",
                "command": "CREATE",
                "params": {
                    "key": "API_KEY",
                    "domain": "localhost",
                    "peer_id": "peer_id",
                    "turn": true
                }
            }"#;
            RequestDto::from_str(message).unwrap()
        };

        // repositoryのMockを生成
        // errorを返してくるケース
        let mut repository = MockRepository::new();
        repository.expect_register().times(1).returning(|_| {
            let answer = error::Error::create_local_error("error");
            return Err(answer);
        });

        // サービスの生成
        let module = GeneralService::builder()
            .with_component_override::<dyn Repository>(Box::new(repository))
            .build();
        let service: &dyn Service = module.resolve_ref();

        // 実行
        let result = service.execute(dto).await;
        if let Err(error::Error::LocalError(message)) = result {
            assert_eq!(message, "error");
        }
    }

    #[tokio::test]
    async fn invalid_parameter() {
        // 間違ったパラメータの生成
        let dto = RequestDto::Test;

        // repositoryのMockを生成
        // 呼ばれないはずである
        let mut repository = MockRepository::new();
        repository
            .expect_register()
            .times(0)
            .returning(|_| unreachable!());

        // サービスの生成
        let module = GeneralService::builder()
            .with_component_override::<dyn Repository>(Box::new(repository))
            .build();
        let service: &dyn Service = module.resolve_ref();

        // 実行
        let result = service.execute(dto).await;
        // 評価
        // 間違ったパラメータである旨を返してくるはずである
        if let Err(error::Error::LocalError(error_message)) = result {
            assert_eq!(error_message, "invalid parameter for GenealService");
        }
    }
}

#[cfg(test)]
mod create_data_test {
    use shaku::HasComponent;

    use super::*;
    use crate::application::dto::request::{DataRequestDto, RequestDto};
    use crate::application::dto::response::{DataResponseDto, ResponseDtoResult};
    use crate::di::GeneralService;
    use crate::domain::entity::response::{Response, ResponseResult};
    use crate::domain::entity::{DataId, SerializableSocket, SocketInfo};
    use crate::domain::repository::MockRepository;

    #[tokio::test]
    async fn success() {
        // CreateDataに成功したメッセージが得られるはずである
        let expected = {
            let socket = SocketInfo::<DataId>::try_create(
                Some("da-06cf1d26-0ef0-4b03-aca6-933027d434c2".to_string()),
                "127.0.0.1",
                10000,
            )
            .unwrap();
            ResponseDtoResult::Success(ResponseDto::Data(DataResponseDto::Create(socket.clone())))
        };

        // CreateDataのパラメータ生成
        let dto = RequestDto::Data(DataRequestDto::Create);

        // repositoryのMockを生成
        // 呼び出しに成功するケース
        let mut repository = MockRepository::new();
        repository.expect_register().times(1).returning(|_| {
            let socket = SocketInfo::<DataId>::try_create(
                Some("da-06cf1d26-0ef0-4b03-aca6-933027d434c2".to_string()),
                "127.0.0.1",
                10000,
            )
            .unwrap();
            Ok(ResponseResult::Success(Response::Data(
                DataResponse::Create(socket),
            )))
        });

        // サービスの生成
        let module = GeneralService::builder()
            .with_component_override::<dyn Repository>(Box::new(repository))
            .build();
        let service: &dyn Service = module.resolve_ref();

        // 実行
        let result = service.execute(dto).await;
        assert_eq!(result.unwrap(), expected);
    }

    #[tokio::test]
    async fn fail() {
        // APIがエラーを返してくるケース

        // CreateDataのパラメータ生成
        let dto = RequestDto::Data(DataRequestDto::Create);

        // repositoryのMockを生成
        // 呼び出しに成功するケース
        let mut repository = MockRepository::new();
        repository.expect_register().times(1).returning(|_| {
            let answer = error::Error::create_local_error("error");
            return Err(answer);
        });

        // サービスの生成
        let module = GeneralService::builder()
            .with_component_override::<dyn Repository>(Box::new(repository))
            .build();
        let service: &dyn Service = module.resolve_ref();

        // 実行
        let result = service.execute(dto).await;
        if let Err(error::Error::LocalError(message)) = result {
            assert_eq!(message, "error");
        }
    }

    #[tokio::test]
    async fn invalid_parameter() {
        // 間違ったパラメータの生成
        let dto = RequestDto::Test;

        // repositoryのMockを生成
        // 呼ばれないはずである
        let mut repository = MockRepository::new();
        repository.expect_register().times(0).returning(|_| {
            let answer = error::Error::create_local_error("error");
            return Err(answer);
        });

        // サービスの生成
        let module = GeneralService::builder()
            .with_component_override::<dyn Repository>(Box::new(repository))
            .build();
        let service: &dyn Service = module.resolve_ref();

        // 実行
        let result = service.execute(dto).await;
        // 間違ったパラメータである旨を返してくるはずである
        if let Err(error::Error::LocalError(error_message)) = result {
            assert_eq!(error_message, "invalid parameter for GenealService");
        }
    }
}
