use async_trait::async_trait;
use tokio::sync::{mpsc, oneshot};

use crate::domain::entity::Stringify;
use crate::domain::entity::{Request, Response};
use crate::domain::repository::Repository;
use crate::error;
use crate::error::Error;

struct RepositoryImpl {
    sender: mpsc::Sender<(oneshot::Sender<String>, String)>,
}

#[async_trait]
impl Repository for RepositoryImpl {
    async fn register(&self, params: Request) -> Result<Response, Error> {
        // SkyWay Crateからの戻り値を得るためのoneshot channelを生成
        let (channel_tx, channel_rx) = tokio::sync::oneshot::channel();

        // Request型である時点でto_stringには失敗しない
        let message = params.to_string().unwrap();

        // SkyWay Crateへメッセージを送る
        // 失敗した場合はエラーメッセージを返す
        if let Err(_) = self.sender.send((channel_tx, message)).await {
            return Err(error::Error::create_local_error(
                "could not send request to skyway crate",
            ));
        }

        // SkyWay Crateからのメッセージを処理する
        match channel_rx.await {
            Ok(message) => Ok(Response::from_str(&message)?),
            Err(_) => Err(error::Error::create_local_error(
                "could not receive response from skyway crate",
            )),
        }
    }
}

#[cfg(test)]
mod infra_test {
    use module::prelude::request_message::{Parameter, PeerServiceParams};

    use super::*;
    use crate::domain::entity::FromStr;

    #[tokio::test]
    async fn success() {
        // 送信メッセージの生成
        let inner = PeerServiceParams::Create {
            params: Parameter(Default::default()),
        };
        let message: Request = Request::Peer(inner);

        // Repository Implの生成
        let (tx, mut rx) = mpsc::channel::<(oneshot::Sender<String>, String)>(10);
        let repository_impl = RepositoryImpl { sender: tx };

        tokio::spawn(async move {
            let (response_tx, request_message) = rx.recv().await.unwrap();

            let request = Request::from_str(&request_message);
            match request {
                Ok(Request::Peer(_)) => {
                    assert!(true)
                }
                _ => {
                    assert!(false)
                }
            }

            let response_str = r#"{
                "is_success":true,
                "result":{
                    "type":"PEER",
                    "command":"CREATE",
                    "peer_id":"hoge",
                    "token":"pt-9749250e-d157-4f80-9ee2-359ce8524308"
                }
            }"#;
            let _ = response_tx.send(response_str.into());
        });

        // 実行
        let result = repository_impl.register(message).await;
        assert!(result.is_ok());
    }

    #[tokio::test]
    // responseが帰ってこないケース
    async fn error_no_response() {
        // 送信メッセージの生成
        let inner = PeerServiceParams::Create {
            params: Parameter(Default::default()),
        };
        let message: Request = Request::Peer(inner);

        // Repository Implの生成
        let (tx, mut rx) = mpsc::channel::<(oneshot::Sender<String>, String)>(10);
        let repository_impl = RepositoryImpl { sender: tx };

        tokio::spawn(async move {
            let (_response_tx, request_message) = rx.recv().await.unwrap();

            let request = Request::from_str(&request_message);
            match request {
                Ok(Request::Peer(_)) => {
                    assert!(true)
                }
                _ => {
                    assert!(false)
                }
            }
        });

        // 実行
        let result = repository_impl.register(message).await;
        match result {
            Err(error::Error::LocalError(message)) => {
                assert_eq!(message, "could not receive response from skyway crate");
            }
            _ => assert!(false),
        }
    }

    #[tokio::test]
    // responseがinvalidなJSONでパースできないケース
    async fn error_recv_invalid_message() {
        // 送信メッセージの生成
        let inner = PeerServiceParams::Create {
            params: Parameter(Default::default()),
        };
        let message: Request = Request::Peer(inner);

        // Repository Implの生成
        let (tx, mut rx) = mpsc::channel::<(oneshot::Sender<String>, String)>(10);
        let repository_impl = RepositoryImpl { sender: tx };

        tokio::spawn(async move {
            let (response_tx, request_message) = rx.recv().await.unwrap();

            let request = Request::from_str(&request_message);
            match request {
                Ok(Request::Peer(_)) => {
                    assert!(true)
                }
                _ => {
                    assert!(false)
                }
            }

            let response_str = r#"{
                "is_success":true,
                "result":{
                    "type":"PEER",
                    "command":"CREATE",
                    "peer_id":"hoge",
                    "token":"pt-9749250e-d157-4f80-9ee2-359ce8524308"
            }"#;
            let _ = response_tx.send(response_str.into());
        });

        // 実行
        let result = repository_impl.register(message).await;
        match result {
            Err(error::Error::SerdeError { error: _ }) => {
                assert!(true)
            }
            _ => assert!(false),
        }
    }
}
