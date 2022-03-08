use async_trait::async_trait;

use crate::application::dto::Dto;
use crate::application::usecase::Service;
use crate::domain::entity::{Request, Response};
use crate::error;
use crate::error::Error;
use crate::Repository;

pub(crate) struct CreatePeer {}

#[async_trait]
impl Service for CreatePeer {
    async fn execute(
        &self,
        repository: &Box<dyn Repository>,
        message: Dto,
    ) -> Result<Response, error::Error> {
        if let Dto::Peer(inner) = message {
            let request = Request::Peer(inner);
            return repository.register(request).await;
        }

        let error_message = format!("wrong parameter {:?}", message);
        return Err(error::Error::create_local_error(&error_message));
    }
}

#[cfg(test)]
mod create_peer_test {
    use super::*;
    use crate::domain::repository::MockRepository;

    #[tokio::test]
    async fn success() {
        // CreatePeerに成功したメッセージが得られるはずである
        let answer = {
            let message = r#"{
                    "is_success":true,
                    "result":{
                        "type":"PEER",
                        "command":"CREATE",
                        "peer_id":"data_caller",
                        "token":"pt-06cf1d26-0ef0-4b03-aca6-933027d434c2"
                    }
                }"#;
            Response::from_str(message).unwrap()
        };

        // CreatePeerのパラメータ生成
        let message = r#"{
            "type": "PEER",
            "command": "CREATE",
            "params": {
                "key": "pt-9749250e-d157-4f80-9ee2-359ce8524308",
                "domain": "localhost",
                "peer_id": "peer_id",
                "turn": true
            }
        }"#;
        let dto = Dto::from_str(message).unwrap();

        // repositoryのMockを生成
        // 呼び出しに成功するケース
        let mut repository = MockRepository::new();
        repository.expect_register().times(1).returning(|_| {
            let message = r#"{
                    "is_success":true,
                    "result":{
                        "type":"PEER",
                        "command":"CREATE",
                        "peer_id":"data_caller",
                        "token":"pt-06cf1d26-0ef0-4b03-aca6-933027d434c2"
                    }
                }"#;
            Response::from_str(message)
        });
        let repository: Box<dyn Repository> = Box::new(repository);

        // 実行
        let create_peer = CreatePeer {};
        let result = create_peer.execute(&repository, dto).await;
        assert_eq!(result.unwrap(), answer);
    }

    #[tokio::test]
    async fn fail() {
        // APIがエラーを返してくるケース

        // CreatePeerのパラメータ生成
        let message = r#"{
            "type": "PEER",
            "command": "CREATE",
            "params": {
                "key": "pt-9749250e-d157-4f80-9ee2-359ce8524308",
                "domain": "localhost",
                "peer_id": "peer_id",
                "turn": true
            }
        }"#;
        let dto = Dto::from_str(message).unwrap();

        // repositoryのMockを生成
        // 呼び出しに成功するケース
        let mut repository = MockRepository::new();
        repository.expect_register().times(1).returning(|_| {
            let answer = error::Error::create_local_error("error");
            return Err(answer);
        });
        let repository: Box<dyn Repository> = Box::new(repository);

        // 実行
        let create_peer = CreatePeer {};
        if let Err(error::Error::LocalError(message)) = create_peer.execute(&repository, dto).await
        {
            assert_eq!(message, "error");
        }
    }

    #[tokio::test]
    async fn invalid_parameter() {
        // 間違ったパラメータの生成
        let dto = Dto::Test;

        // mockの生成
        // パラメータが違う場合、repositoryが呼ばれないはずである
        let mut repository = MockRepository::new();
        repository
            .expect_register()
            .times(0)
            .returning(|_| unreachable!());
        let repository: Box<dyn Repository> = Box::new(repository);

        // 実行
        let create_peer = CreatePeer {};

        // 評価
        // 間違ったパラメータである旨を返してくるはずである
        if let Err(error::Error::LocalError(error_message)) =
            create_peer.execute(&repository, dto).await
        {
            assert_eq!(error_message, "wrong parameter Test");
        }
    }
}
