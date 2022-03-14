// このサービスでは、End-User-Programの指示を受けて、MediaConnectionの確立要求を行う
// 責務は以下の通りである
// 1. GWにMedia Portを開放させる。これはVideo, Audioともに行う
// 2. CALL APIをコールし、MediaConnectionの確立を開始する
//
// WebRTC GWの仕様により、確立は受信側でAnswerが行われたタイミングである。
// このサービスではあくまで確立要求のみを行う。
// 実際にMediaConnectionが確立されたかどうか知るために、End-User-ProgramはCONNECT Eventを監視する必要がある

use async_trait::async_trait;

use crate::application::dto::request::{MediaRequestDto, RequestDto};
use crate::application::dto::response::{
    CallResponseDto, MediaInfo, MediaPair, MediaResponseDto, ResponseDto, ResponseDtoResult,
};
use crate::application::usecase::Service;
use crate::domain::entity::request::{IsVideo, MediaRequest, Request};
use crate::domain::entity::response::{MediaResponse, Response, ResponseResult};
use crate::domain::entity::{MediaId, PhantomId, RtcpId, SerializableSocket, SocketInfo};
use crate::{
    error, get_media_connection_state, CallbackFunctions, Logger, ProgramState, Repository,
};

pub(crate) struct Answer {}

impl Default for Answer {
    fn default() -> Self {
        Answer {}
    }
}

async fn create_media(
    repository: &Box<dyn Repository>,
    program_state: &ProgramState,
    logger: &Logger,
    is_video: bool,
) -> Result<SocketInfo<MediaId>, error::Error> {
    let message = Request::Media(MediaRequest::ContentCreate {
        params: IsVideo { is_video },
    });
    match repository.register(program_state, logger, message).await {
        Ok(ResponseResult::Success(Response::Media(MediaResponse::ContentCreate(socket)))) => {
            Ok(socket)
        }
        Err(e) => Err(e),
        _ => Err(error::Error::create_local_error(
            "error in create Video Object",
        )),
    }
}

async fn create_rtcp(
    repository: &Box<dyn Repository>,
    program_state: &ProgramState,
    logger: &Logger,
    param: Option<()>,
) -> Result<SocketInfo<RtcpId>, error::Error> {
    let message = Request::Media(MediaRequest::RtcpCreate { params: param });
    match repository.register(program_state, logger, message).await {
        Ok(ResponseResult::Success(Response::Media(MediaResponse::RtcpCreate(socket)))) => {
            Ok(socket)
        }
        Err(e) => Err(e),
        _ => Err(error::Error::create_local_error(
            "error in create Video Object",
        )),
    }
}

#[async_trait]
impl Service for Answer {
    async fn execute(
        &self,
        repository: &Box<dyn Repository>,
        program_state: &ProgramState,
        logger: &Logger,
        _cb_functions: &CallbackFunctions,
        message: RequestDto,
    ) -> Result<ResponseDtoResult, error::Error> {
        let log = format!(
            "Call Service starting. Parameter: {:?}",
            message.to_string()
        );
        logger.debug(log.as_str());

        if let RequestDto::Media(MediaRequestDto::Answer { params }) = message {
            let video_send_socket = create_media(repository, program_state, logger, true).await?;
            let video_rtcp_send_socket =
                create_rtcp(repository, program_state, logger, Some(())).await?;
            let audio_send_socket = create_media(repository, program_state, logger, false).await?;
            let audio_rtcp_send_socket =
                create_rtcp(repository, program_state, logger, None).await?;

            let video = {
                let send = {
                    let media = video_send_socket;
                    let rtcp = video_rtcp_send_socket;
                    MediaPair { media, rtcp }
                };

                let recv = {
                    let media =
                        SocketInfo::<PhantomId>::try_create(None, "127.0.0.1", 20000).unwrap();
                    let rtcp =
                        SocketInfo::<PhantomId>::try_create(None, "127.0.0.1", 20001).unwrap();
                    MediaPair { media, rtcp }
                };

                MediaInfo { send, recv }
            };

            let audio = {
                let send = {
                    let media = audio_send_socket;
                    let rtcp = audio_rtcp_send_socket;
                    MediaPair { media, rtcp }
                };

                let recv = {
                    let media =
                        SocketInfo::<PhantomId>::try_create(None, "127.0.0.1", 20010).unwrap();
                    let rtcp =
                        SocketInfo::<PhantomId>::try_create(None, "127.0.0.1", 20011).unwrap();
                    MediaPair { media, rtcp }
                };

                MediaInfo { send, recv }
            };

            let request = Request::Media(MediaRequest::Answer { params });
            if let ResponseResult::Success(Response::Media(MediaResponse::Answer(answer_result))) =
                repository.register(program_state, logger, request).await?
            {
                let call_response = CallResponseDto {
                    video,
                    audio,
                    media_connection_id: answer_result.media_connection_id.clone(),
                };
                get_media_connection_state()
                    .lock()
                    .unwrap()
                    .insert(answer_result.media_connection_id.clone(), call_response);

                return Ok(ResponseDtoResult::Success(ResponseDto::Media(
                    MediaResponseDto::Answer(answer_result),
                )));
            }
        }

        return Err(error::Error::create_local_error(
            "invalid message in call service",
        ));
    }
}

#[cfg(test)]
mod answer_media_test {
    use std::collections::HashMap;
    use std::sync::Mutex;

    use super::*;
    use crate::application::dto::request::MediaRequestDto;
    use crate::application::dto::response::CallResponseDto;
    use crate::application::usecase::helper;
    use crate::domain::entity::request::{AnswerParameters, MediaRequest, Request};
    use crate::domain::entity::response::ResponseResult;
    use crate::domain::entity::{
        AnswerQuery, AnswerResult, Constraints, MediaConnectionId, SocketInfo,
    };
    use crate::domain::repository::MockRepository;

    #[tokio::test]
    // eventとして異常な文字列を受信した場合
    async fn success() {
        let _ = crate::MEDIA_CONNECTION_STATE_INSTANCE.set(Mutex::new(HashMap::new()));

        // 正解データの生成
        let dto = {
            //CallResponseDtoはVideo, Audio, MediaConnectionIdの情報が必要なので生成する
            let video = {
                let send = {
                    let media = SocketInfo::<MediaId>::try_create(
                        Some("vi-4d053831-5dc2-461b-a358-d062d6115216".to_string()),
                        "127.0.0.1",
                        10000,
                    )
                    .unwrap();
                    let rtcp = SocketInfo::<RtcpId>::try_create(
                        Some("rc-4d053831-5dc2-461b-a358-d062d6115216".to_string()),
                        "127.0.0.1",
                        10001,
                    )
                    .unwrap();
                    MediaPair { media, rtcp }
                };

                let recv = {
                    let media =
                        SocketInfo::<PhantomId>::try_create(None, "127.0.0.1", 20000).unwrap();
                    let rtcp =
                        SocketInfo::<PhantomId>::try_create(None, "127.0.0.1", 20001).unwrap();
                    MediaPair { media, rtcp }
                };

                MediaInfo { send, recv }
            };

            let audio = {
                let send = {
                    let media = SocketInfo::<MediaId>::try_create(
                        Some("au-4d053831-5dc2-461b-a358-d062d6115216".to_string()),
                        "127.0.0.1",
                        10010,
                    )
                    .unwrap();
                    let rtcp = SocketInfo::<RtcpId>::try_create(
                        Some("rc-5d053831-5dc2-461b-a358-d062d6115216".to_string()),
                        "127.0.0.1",
                        10011,
                    )
                    .unwrap();
                    MediaPair { media, rtcp }
                };

                let recv = {
                    let media =
                        SocketInfo::<PhantomId>::try_create(None, "127.0.0.1", 20010).unwrap();
                    let rtcp =
                        SocketInfo::<PhantomId>::try_create(None, "127.0.0.1", 20011).unwrap();
                    MediaPair { media, rtcp }
                };

                MediaInfo { send, recv }
            };

            let media_connection_id =
                MediaConnectionId::try_create("mc-102127d9-30de-413b-93f7-41a33e39d82b").unwrap();

            CallResponseDto {
                video,
                audio,
                media_connection_id,
            }
        };
        let answer = ResponseDtoResult::Success(ResponseDto::Media(MediaResponseDto::Answer(
            AnswerResult {
                media_connection_id: dto.media_connection_id.clone(),
                send_sockets: None,
                recv_sockets: None,
            },
        )));

        // repositoryのMockを生成
        // answerに成功するケース
        let mut repository = MockRepository::new();
        repository
            .expect_register()
            // create dataで失敗した場合は1回しか呼ばれない
            .returning(move |_, _, request| match request {
                Request::Media(MediaRequest::ContentCreate { params }) => {
                    if params.is_video {
                        Ok(ResponseResult::Success(Response::Media(
                            MediaResponse::ContentCreate(dto.video.send.media.clone()),
                        )))
                    } else {
                        Ok(ResponseResult::Success(Response::Media(
                            MediaResponse::ContentCreate(dto.audio.send.media.clone()),
                        )))
                    }
                }
                Request::Media(MediaRequest::RtcpCreate { params }) => {
                    // 本来paramsとしてはNoneが飛ぶが、テストの便宜上Some(())が来たらVideo用の情報を返している
                    if params.is_some() {
                        Ok(ResponseResult::Success(Response::Media(
                            MediaResponse::RtcpCreate(dto.video.send.rtcp.clone()),
                        )))
                    } else {
                        Ok(ResponseResult::Success(Response::Media(
                            MediaResponse::RtcpCreate(dto.audio.send.rtcp.clone()),
                        )))
                    }
                }
                Request::Media(MediaRequest::Answer { params }) => Ok(ResponseResult::Success(
                    Response::Media(MediaResponse::Answer(AnswerResult {
                        media_connection_id: params.media_connection_id,
                        send_sockets: None,
                        recv_sockets: None,
                    })),
                )),
                _ => todo!(),
            });
        let repository: Box<dyn Repository> = Box::new(repository);

        let logger = helper::create_logger();
        let program_state = helper::create_program_state();
        let function = helper::create_functions();
        let params = AnswerParameters {
            media_connection_id: dto.media_connection_id.clone(),
            answer_query: AnswerQuery {
                constraints: Constraints {
                    video: false,
                    videoReceiveEnabled: None,
                    audio: false,
                    audioReceiveEnabled: None,
                    video_params: None,
                    audio_params: None,
                    metadata: None,
                },
                redirect_params: None,
            },
        };

        let answer_service = Answer::default();
        let result = answer_service
            .execute(
                &repository,
                &program_state,
                &logger,
                &function,
                RequestDto::Media(MediaRequestDto::Answer { params }),
            )
            .await;

        assert_eq!(result.unwrap(), answer);
    }
}
