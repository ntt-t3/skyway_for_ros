use std::sync::Arc;

use async_trait::async_trait;
use shaku::Component;
use skyway_webrtc_gateway_caller::prelude::response_parser::{DataResponse, ResponseResult};

use crate::application::dto::request::{DataRequestDto, RequestDto};
use crate::application::dto::response::{DataResponseDto, ResponseDto, ResponseDtoResult};
use crate::application::factory::Factory;
use crate::application::usecase::Service;
use crate::domain::entity::request::{DataRequest, Request};
use crate::domain::entity::response::Response;
use crate::domain::repository::Repository;
use crate::error;

#[derive(Component)]
#[shaku(interface = Service)]
pub(crate) struct CreateData {
    #[shaku(inject)]
    repository: Arc<dyn Repository>,
}

#[async_trait]
impl Service for CreateData {
    async fn execute(&self, request: RequestDto) -> Result<ResponseDtoResult, error::Error> {
        match request {
            RequestDto::Data(DataRequestDto::Create) => {
                let request = Request::Data(DataRequest::Create { params: false });
                let result = self.repository.register(request).await?;

                // 成功した場合はC++側にpeer_id, tokenを渡す
                match result {
                    ResponseResult::Success(Response::Data(DataResponse::Create(socket))) => {
                        return Ok(ResponseDtoResult::Success(ResponseDto::Data(
                            DataResponseDto::Create(socket),
                        )));
                    }
                    // API Callには成功したが、内部処理に失敗したケース
                    ResponseResult::Error(message) => {
                        return Ok(ResponseDtoResult::Error(message));
                    }
                    _ => {
                        unreachable!()
                    }
                }
            }
            _ => {
                let error_message = format!("wrong parameter {:?}", request);
                return Err(error::Error::create_local_error(&error_message));
            }
        }
    }
}

#[cfg(test)]
mod create_data_test {
    use shaku::HasComponent;

    use super::*;
    use crate::application::dto::request::{DataRequestDto, RequestDto};
    use crate::application::dto::response::{DataResponseDto, ResponseDtoResult};
    use crate::di::DataCreateService;
    use crate::domain::entity::response::ResponseResult;
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
        let module = DataCreateService::builder()
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
        let module = DataCreateService::builder()
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
        let module = DataCreateService::builder()
            .with_component_override::<dyn Repository>(Box::new(repository))
            .build();
        let service: &dyn Service = module.resolve_ref();

        // 実行
        let result = service.execute(dto).await;
        // 間違ったパラメータである旨を返してくるはずである
        if let Err(error::Error::LocalError(error_message)) = result {
            assert_eq!(error_message, "wrong parameter Test");
        }
    }
}
