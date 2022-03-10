use std::ffi::CString;
// このサービスでは、End-User-Programの指示を受けて、DataConnectionの確立を行う
// 責務は以下の通りである
// 1. GWにData Portを開放させる
// 2. Data Portにデータを流し込むためのSrc Topicのパラメータを生成する
// 3. GWから受信したデータをEnd-User-Programにデータを渡すためのDest Topicのパラメータを生成する
// 4. CONNECT APIをコールし、DataConnectionを確立させる
// 5. 4.で確立に成功した場合は、C++側の機能を利用し、Src, Dest Topic生成、保存する
use std::net::TcpListener;

use async_trait::async_trait;
use module::prelude::{PhantomId, SocketInfo};

use crate::application::dto::{
    DataDtoResponseMessageBodyEnum, DataRequestDtoParams, RequestDto, ResponseDto,
    ResponseDtoMessageBodyEnum,
};
use crate::application::usecase::Service;
use crate::application::Functions;
use crate::application::{DestinationParameters, SourceParameters, TopicParameters};
use crate::domain::entity::{
    ConnectQuery, DataId, DataIdWrapper, DataRequestParams, DataResponseMessageBodyEnum,
    ResponseMessageBodyEnum,
};
use crate::domain::entity::{Request, Response, SerializableId, SerializableSocket};
use crate::{error, Logger, ProgramState, Repository};

fn available_port() -> std::io::Result<u16> {
    match TcpListener::bind("0.0.0.0:0") {
        Ok(listener) => Ok(listener.local_addr().unwrap().port()),
        Err(e) => Err(e),
    }
}

pub(crate) struct Connect {}

impl Default for Connect {
    fn default() -> Self {
        Connect {}
    }
}

async fn create_data(
    repository: &Box<dyn Repository>,
    program_state: &ProgramState,
    logger: &Logger,
) -> Result<(DataId, String, u16), error::Error> {
    logger.debug("create_data for DATA CONNECT");
    let request = Request::Data(DataRequestParams::Create {});

    return match repository.register(program_state, logger, request).await? {
        Response::Success(ResponseMessageBodyEnum::Data(DataResponseMessageBodyEnum::Create(
            ref socket_info,
        ))) => {
            let data_id = socket_info.get_id().unwrap();
            let address = socket_info.ip().to_string();
            let port = socket_info.port();
            Ok((data_id, address, port))
        }
        _ => Err(error::Error::create_local_error("invalid response")),
    };
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
    ) -> Result<ResponseDto, error::Error> {
        let log = format!(
            "Connect Service starting. Parameter: {:?}",
            message.to_string()
        );
        logger.debug(log.as_str());

        if let RequestDto::Data(DataRequestDtoParams::Connect {
            params: connect_params,
        }) = message
        {
            // 1.は単独で実施可能なので最初に行う
            let (data_id, address, port) = create_data(repository, program_state, logger).await?;

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
            let params = Request::Data(DataRequestParams::Connect { params });
            let result = repository.register(program_state, logger, params).await?;

            // 5.でSrc, Dest Topicを保存する際には、DataConnection IDをキーにしたhashで管理するため、
            // 4.の実施後である必要がある
            if let Response::Success(ResponseMessageBodyEnum::Data(
                DataResponseMessageBodyEnum::Connect(params),
            )) = result
            {
                // 2.はData Port開放時に得られるData IDをTopic Nameにするので、1.の後に実施する
                let source_parameters = SourceParameters {
                    source_topic_name: CString::new(data_id.as_str()).unwrap().into_raw(),
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

                return Ok(ResponseDto::Success(ResponseDtoMessageBodyEnum::Data(
                    DataDtoResponseMessageBodyEnum::Connect(params),
                )));
            }
        }

        return Err(error::Error::create_local_error("invalid parameters"));
    }
}

#[cfg(test)]
mod connect_data_test {
    use super::*;
    use crate::application::dto::ConnectParams;
    use crate::application::usecase::helper;
    use crate::domain::entity::{
        DataConnectionId, DataConnectionIdWrapper, PeerId, SocketInfo, Token,
    };
    use crate::domain::repository::MockRepository;

    #[tokio::test]
    // eventとして異常な文字列を受信した場合
    async fn success() {
        // 成功するケースの結果を生成
        let answer_str = r#"{
            "is_success":true,
            "result":{
                "type":"DATA",
                "command":"CONNECT",
                "data_connection_id":"dc-4995f372-fb6a-4196-b30a-ce11e5c7f56c"
            }
        }"#;
        let answer = ResponseDto::from_str(answer_str).unwrap();

        // repositoryのMockを生成
        // create_dataに失敗するケース
        let mut repository = MockRepository::new();
        repository
            .expect_register()
            .times(2)
            .returning(|_, _, dto| {
                let message = r#"{
                    "data_id": "da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
                    "port": 10001,
                    "ip_v4": "127.0.0.1"
                }"#;
                let socket = serde_json::from_str::<SocketInfo<DataId>>(message).unwrap();
                return match dto {
                    Request::Data(DataRequestParams::Create { .. }) => Ok(Response::Success(
                        ResponseMessageBodyEnum::Data(DataResponseMessageBodyEnum::Create(socket)),
                    )),
                    Request::Data(DataRequestParams::Connect { .. }) => {
                        Ok(Response::Success(ResponseMessageBodyEnum::Data(
                            DataResponseMessageBodyEnum::Connect(DataConnectionIdWrapper {
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
        let logger = helper::create_logger();
        let program_state = helper::create_program_state();
        let function = helper::create_functions();

        let connect = Connect::default();
        let param = RequestDto::Data(DataRequestDtoParams::Connect {
            params: ConnectParams {
                peer_id: PeerId::new("peer_id"),
                token: Token::try_create("pt-9749250e-d157-4f80-9ee2-359ce8524308").unwrap(),
                target_id: PeerId::new("target_id"),
                destination_topic: "topic".to_string(),
            },
        });

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
            .times(1)
            .returning(|_, _, _| Ok(Response::Error("invalid".to_string())));
        let repository: Box<dyn Repository> = Box::new(repository);
        let logger = helper::create_logger();
        let program_state = helper::create_program_state();
        let function = helper::create_functions();

        let connect = Connect::default();
        let param = RequestDto::Data(DataRequestDtoParams::Connect {
            params: ConnectParams {
                peer_id: PeerId::new("peer_id"),
                token: Token::try_create("pt-9749250e-d157-4f80-9ee2-359ce8524308").unwrap(),
                target_id: PeerId::new("target_id"),
                destination_topic: "topic".to_string(),
            },
        });
        let _response = connect
            .execute(&repository, &program_state, &logger, &function, param)
            .await;
    }
}
