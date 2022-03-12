// このサービスでは、End-User-Programの指示を受けて、DataConnectionの確立を行う
// 責務は以下の通りである
// 1. GWにData Portを開放させる
// 2. Data Portにデータを流し込むためのSrc Topicのパラメータを生成する
// 3. GWから受信したデータをEnd-User-Programにデータを渡すためのDest Topicのパラメータを生成する
// 4. CONNECT APIをコールし、DataConnectionを確立させる
// 5. 4.で確立に成功した場合は、C++側の機能を利用し、Src, Dest Topic生成、保存する

use std::ffi::CString;

use async_trait::async_trait;

use crate::application::dto::request::{DataRequestDto, RequestDto};
use crate::application::dto::response::{
    DataConnectionResponse, DataResponseDto, ResponseDto, ResponseDtoResult,
};
use crate::application::usecase::data::create_data;
use crate::application::usecase::{available_port, Service};
use crate::application::Functions;
use crate::application::{DestinationParameters, SourceParameters, TopicParameters};
use crate::domain::entity::request::{DataRequest, Request};
use crate::domain::entity::response::{DataResponse, Response, ResponseResult};
use crate::domain::entity::{ConnectQuery, DataIdWrapper, PhantomId, SocketInfo};
use crate::domain::entity::{SerializableId, SerializableSocket};
use crate::{error, Logger, ProgramState, Repository};

pub(crate) struct Connect {}

impl Default for Connect {
    fn default() -> Self {
        Connect {}
    }
}

#[async_trait]
impl Service for Connect {
    async fn execute(
        &self,
        repository: &Box<dyn Repository>,
        program_state: &ProgramState,
        logger: &Logger,
        cb_functions: &Functions,
        message: RequestDto,
    ) -> Result<ResponseDtoResult, error::Error> {
        let log = format!(
            "Connect Service starting. Parameter: {:?}",
            message.to_string()
        );
        logger.debug(log.as_str());

        if let RequestDto::Data(DataRequestDto::Connect {
            params: connect_params,
        }) = message
        {
            // 1.は単独で実施可能なので最初に行う
            let (data_id, address, port) = create_data(repository, program_state, logger).await?;
            // topic名には-が使えないので_に置換する
            let source_topic_name = data_id.as_str().replace("-", "_");

            // 3. GWから受信したデータをEnd-User-Programにデータを渡すためのDest Topicのパラメータを生成する
            // GWからこのポートに転送されたデータが最終的にエンドユーザに届けられる
            let available_port = available_port().expect("bind port failed");

            // 4.でCONNECT APIを呼ぶ際には、JSONメッセージ内にData PortのData IDと、
            // Dest ObjectのUDPソケット情報が必要なので、このタイミングで実施する
            let query = ConnectQuery {
                peer_id: connect_params.peer_id,
                token: connect_params.token,
                options: None,
                target_id: connect_params.target_id,
                params: Some(DataIdWrapper {
                    data_id: data_id.clone(),
                }),
                redirect_params: Some(
                    SocketInfo::<PhantomId>::try_create(None, "127.0.0.1", available_port).unwrap(),
                ),
            };
            let params = serde_json::from_str::<ConnectQuery>(
                serde_json::to_string(&query).unwrap().as_str(),
            )
            .unwrap();
            let params = Request::Data(DataRequest::Connect { params });
            let result = repository.register(program_state, logger, params).await?;

            // 5.でSrc, Dest Topicを保存する際には、DataConnection IDをキーにしたhashで管理するため、
            // 4.の実施後である必要がある
            if let ResponseResult::Success(Response::Data(DataResponse::Connect(params))) = result {
                // 2.はData Port開放時に得られるData IDをTopic Nameにするので、1.の後に実施する
                let source_parameters = SourceParameters {
                    source_topic_name: CString::new(source_topic_name.as_str()).unwrap().into_raw(),
                    destination_address: CString::new(address.as_str()).unwrap().into_raw(),
                    destination_port: port,
                };

                // 3.はDto内に含まれるEnd-User-ProgramのTopic Nameがあればいつでも実施可能である
                let destination_parameters = DestinationParameters {
                    source_port: available_port,
                    destination_topic_name: CString::new(connect_params.destination_topic.as_str())
                        .unwrap()
                        .into_raw(),
                };

                let topic_parameters = TopicParameters {
                    data_connection_id: CString::new(params.data_connection_id.as_str())
                        .unwrap()
                        .into_raw(),
                    source_parameters,
                    destination_parameters,
                };

                // 5. 4.で確立に成功した場合は、C++側の機能を利用し、Src, Dest Topic生成、保存する
                cb_functions.data_callback(topic_parameters);

                let response_data = DataConnectionResponse {
                    data_connection_id: params.data_connection_id,
                    source_topic_name: source_topic_name,
                    source_ip: address,
                    source_port: port,
                    destination_topic_name: connect_params.destination_topic,
                };
                return Ok(ResponseDtoResult::Success(ResponseDto::Data(
                    DataResponseDto::Connect(response_data),
                )));
            }
        }

        return Err(error::Error::create_local_error("invalid parameters"));
    }
}

#[cfg(test)]
mod connect_data_test {
    use super::*;
    use crate::application::dto::request::ConnectDtoParams;
    use crate::application::usecase::helper;
    use crate::domain::entity::{
        DataConnectionId, DataConnectionIdWrapper, DataId, PeerId, SocketInfo, Token,
    };
    use crate::domain::repository::MockRepository;

    #[tokio::test]
    // eventとして異常な文字列を受信した場合
    async fn success() {
        // 成功するケースの結果を生成
        let value = DataConnectionResponse {
            data_connection_id: DataConnectionId::try_create(
                "dc-4995f372-fb6a-4196-b30a-ce11e5c7f56c",
            )
            .unwrap(),
            source_topic_name: "da_50a32bab_b3d9_4913_8e20_f79c90a6a211".to_string(),
            source_ip: "127.0.0.1".to_string(),
            source_port: 10000,
            destination_topic_name: "destination_topic".to_string(),
        };
        let answer = ResponseDtoResult::Success(ResponseDto::Data(DataResponseDto::Connect(value)));

        // repositoryのMockを生成
        // create_dataに失敗するケース
        let mut repository = MockRepository::new();
        repository
            .expect_register()
            .times(2)
            .returning(|_, _, dto| {
                return match dto {
                    Request::Data(DataRequest::Create { .. }) => {
                        // create_dataのmock
                        // 成功し、ポートを返すケース
                        let message = r#"{
                            "data_id": "da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
                            "port": 10000,
                            "ip_v4": "127.0.0.1"
                        }"#;
                        let socket = serde_json::from_str::<SocketInfo<DataId>>(message).unwrap();
                        Ok(ResponseResult::Success(Response::Data(
                            DataResponse::Create(socket),
                        )))
                    }
                    Request::Data(DataRequest::Connect { .. }) => {
                        // connectのmock
                        // 成功し、DataConnectionIdを返すケース
                        Ok(ResponseResult::Success(Response::Data(
                            DataResponse::Connect(DataConnectionIdWrapper {
                                data_connection_id: DataConnectionId::try_create(
                                    "dc-4995f372-fb6a-4196-b30a-ce11e5c7f56c",
                                )
                                .unwrap(),
                            }),
                        )))
                    }
                    _ => {
                        unreachable!()
                    }
                };
            });
        let repository: Box<dyn Repository> = Box::new(repository);

        // パラメータの生成
        let logger = helper::create_logger();
        let program_state = helper::create_program_state();
        let function = helper::create_functions();
        let param = RequestDto::Data(DataRequestDto::Connect {
            params: ConnectDtoParams {
                peer_id: PeerId::new("peer_id"),
                token: Token::try_create("pt-9749250e-d157-4f80-9ee2-359ce8524308").unwrap(),
                target_id: PeerId::new("target_id"),
                destination_topic: "destination_topic".to_string(),
            },
        });

        // 実行
        let connect = Connect::default();
        let response = connect
            .execute(&repository, &program_state, &logger, &function, param)
            .await
            .unwrap();
        assert_eq!(response, answer);
    }

    #[tokio::test]
    // eventとして異常な文字列を受信した場合
    async fn create_data_error() {
        // repositoryのMockを生成
        // create_dataに失敗するケース
        let mut repository = MockRepository::new();
        repository
            .expect_register()
            // create dataで失敗した場合は1回しか呼ばれない
            .times(1)
            .returning(|_, _, _| Ok(ResponseResult::Error("invalid".to_string())));
        let repository: Box<dyn Repository> = Box::new(repository);

        // パラメータの生成
        let logger = helper::create_logger();
        let program_state = helper::create_program_state();
        let function = helper::create_functions();
        let param = RequestDto::Data(DataRequestDto::Connect {
            params: ConnectDtoParams {
                peer_id: PeerId::new("peer_id"),
                token: Token::try_create("pt-9749250e-d157-4f80-9ee2-359ce8524308").unwrap(),
                target_id: PeerId::new("target_id"),
                destination_topic: "topic".to_string(),
            },
        });

        // 実行
        let connect = Connect::default();
        let _response = connect
            .execute(&repository, &program_state, &logger, &function, param)
            .await;
    }
}
