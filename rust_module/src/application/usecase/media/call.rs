// このサービスでは、End-User-Programの指示を受けて、MediaConnectionの確立要求を行う
// 責務は以下の通りである
// 1. GWにMedia Portを開放させる。これはVideo, Audioともに行う
// 2. CALL APIをコールし、MediaConnectionの確立を開始する
//
// WebRTC GWの仕様により、確立は受信側でAnswerが行われたタイミングである。
// このサービスではあくまで確立要求のみを行う。
// 実際にMediaConnectionが確立されたかどうか知るために、End-User-ProgramはCONNECT Eventを監視する必要がある

use async_trait::async_trait;

use crate::application::dto::request::{ConstraintsDto, MediaRequestDto, RequestDto};
use crate::application::dto::response::{
    CallResponseDto, MediaPair, MediaResponseDto, ResponseDto, ResponseDtoResult, SendParams,
};
use crate::application::usecase::media::*;
use crate::application::usecase::Service;
use crate::domain::entity::request::{MediaRequest, Request};
use crate::domain::entity::response::{MediaResponse, Response, ResponseResult};
use crate::domain::entity::{
    CallQuery, Constraints, MediaConnectionIdWrapper, MediaId, MediaParams, RedirectParameters,
    RtcpId, SerializableSocket,
};
use crate::{
    error, get_media_connection_state, CallbackFunctions, Logger, ProgramState, Repository,
};

pub(crate) struct Call {}

impl Default for Call {
    fn default() -> Self {
        Call {}
    }
}

pub(crate) fn create_constraint(
    video_id: MediaId,
    video_rtcp_id: RtcpId,
    audio_id: MediaId,
    audio_rtcp_id: RtcpId,
    constraint_dto: &Option<ConstraintsDto>,
    redirect_params: &Option<RedirectParameters>,
) -> Constraints {
    let video_receive_enabled = if let Some(RedirectParameters {
        video: Some(ref _video),
        ..
    }) = redirect_params
    {
        Some(true)
    } else {
        None
    };

    let audio_receive_enabled = if let Some(RedirectParameters {
        audio: Some(ref _audio),
        ..
    }) = redirect_params
    {
        Some(true)
    } else {
        None
    };

    let video_params = if let Some(ConstraintsDto {
        video_params: Some(ref params),
        ..
    }) = constraint_dto
    {
        Some(MediaParams {
            band_width: params.band_width,
            codec: params.codec.clone(),
            media_id: video_id,
            rtcp_id: Some(video_rtcp_id),
            payload_type: params.payload_type,
            sampling_rate: params.sampling_rate,
        })
    } else {
        None
    };

    let audio_params = if let Some(ConstraintsDto {
        audio_params: Some(ref params),
        ..
    }) = constraint_dto
    {
        Some(MediaParams {
            band_width: params.band_width,
            codec: params.codec.clone(),
            media_id: audio_id,
            rtcp_id: Some(audio_rtcp_id),
            payload_type: params.payload_type,
            sampling_rate: params.sampling_rate,
        })
    } else {
        None
    };

    let metadata = if let Some(ConstraintsDto {
        metadata: Some(ref metadata),
        ..
    }) = constraint_dto
    {
        Some(metadata.clone())
    } else {
        None
    };

    Constraints {
        video: true,
        videoReceiveEnabled: video_receive_enabled,
        audio: true,
        audioReceiveEnabled: audio_receive_enabled,
        video_params: video_params,
        audio_params: audio_params,
        metadata: metadata,
    }
}

#[async_trait]
impl Service for Call {
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

        if let RequestDto::Media(MediaRequestDto::Call { params }) = message {
            let video_socket = create_media(repository, program_state, logger, true).await?;
            let video_rtcp_socket =
                create_rtcp(repository, program_state, logger, Some(())).await?;
            let audio_socket = create_media(repository, program_state, logger, false).await?;
            let audio_rtcp_socket = create_rtcp(repository, program_state, logger, None).await?;
            // Readyイベントでユーザに返すために保持
            let send_params = SendParams {
                video: MediaPair {
                    media: video_socket.clone(),
                    rtcp: video_rtcp_socket.clone(),
                },
                audio: MediaPair {
                    media: audio_socket.clone(),
                    rtcp: audio_rtcp_socket.clone(),
                },
            };
            let redirect_params = params.redirect_params.clone();
            let constraints = create_constraint(
                video_socket.get_id().unwrap(),
                video_rtcp_socket.get_id().unwrap(),
                audio_socket.get_id().unwrap(),
                audio_rtcp_socket.get_id().unwrap(),
                &params.constraints,
                &params.redirect_params,
            );
            let params = CallQuery {
                peer_id: params.peer_id,
                token: params.token,
                target_id: params.target_id,
                constraints: Some(constraints),
                redirect_params: redirect_params.clone(),
            };

            let request = Request::Media(MediaRequest::Call { params });
            let result = repository.register(program_state, logger, request).await?;

            match result {
                ResponseResult::Success(Response::Media(MediaResponse::Call(wrapper))) => {
                    let call_response = CallResponseDto {
                        send_params,
                        redirect_params,
                        media_connection_id: wrapper.media_connection_id.clone(),
                    };
                    get_media_connection_state()
                        .lock()
                        .unwrap()
                        .insert(wrapper.media_connection_id.clone(), call_response);

                    return Ok(ResponseDtoResult::Success(ResponseDto::Media(
                        MediaResponseDto::Call(MediaConnectionIdWrapper {
                            media_connection_id: wrapper.media_connection_id,
                        }),
                    )));
                }
                ResponseResult::Error(message) => return Ok(ResponseDtoResult::Error(message)),
                _ => {
                    unreachable!()
                }
            }
        }

        return Err(error::Error::create_local_error(
            "invalid message in call service",
        ));
    }
}

#[cfg(test)]
mod call_media_test {
    use std::collections::HashMap;
    use std::sync::Mutex;

    use super::*;
    use crate::application::dto::request::{CallQueryDto, MediaRequestDto};
    use crate::application::usecase::helper;
    use crate::domain::entity::request::{MediaRequest, Request};
    use crate::domain::entity::response::ResponseResult;
    use crate::domain::entity::{MediaConnectionId, PeerId, SocketInfo, Token};
    use crate::domain::repository::MockRepository;

    #[tokio::test]
    // eventとして異常な文字列を受信した場合
    async fn success() {
        let _ = crate::MEDIA_CONNECTION_STATE_INSTANCE.set(Mutex::new(HashMap::new()));

        // 正解データの生成
        let dto = {
            //CallResponseDtoはVideo, Audio, MediaConnectionIdの情報が必要なので生成する
            let send = {
                let video = {
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

                let audio = {
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

                SendParams { video, audio }
            };

            let media_connection_id =
                MediaConnectionId::try_create("mc-102127d9-30de-413b-93f7-41a33e39d82b").unwrap();

            CallResponseDto {
                send_params: send,
                redirect_params: None,
                media_connection_id,
            }
        };
        let answer = ResponseDtoResult::Success(ResponseDto::Media(MediaResponseDto::Call(
            MediaConnectionIdWrapper {
                media_connection_id: dto.media_connection_id.clone(),
            }
            .clone(),
        )));

        // repositoryのMockを生成
        // create_dataに失敗するケース
        let mut repository = MockRepository::new();
        repository
            .expect_register()
            // create dataで失敗した場合は1回しか呼ばれない
            .returning(move |_, _, request| match request {
                Request::Media(MediaRequest::ContentCreate { params }) => {
                    if params.is_video {
                        Ok(ResponseResult::Success(Response::Media(
                            MediaResponse::ContentCreate(dto.send_params.video.media.clone()),
                        )))
                    } else {
                        Ok(ResponseResult::Success(Response::Media(
                            MediaResponse::ContentCreate(dto.send_params.audio.media.clone()),
                        )))
                    }
                }
                Request::Media(MediaRequest::RtcpCreate { params }) => {
                    // 本来paramsとしてはNoneが飛ぶが、テストの便宜上Some(())が来たらVideo用の情報を返している
                    if params.is_some() {
                        Ok(ResponseResult::Success(Response::Media(
                            MediaResponse::RtcpCreate(dto.send_params.video.rtcp.clone()),
                        )))
                    } else {
                        Ok(ResponseResult::Success(Response::Media(
                            MediaResponse::RtcpCreate(dto.send_params.audio.rtcp.clone()),
                        )))
                    }
                }
                Request::Media(MediaRequest::Call { params: _ }) => Ok(ResponseResult::Success(
                    Response::Media(MediaResponse::Call(MediaConnectionIdWrapper {
                        media_connection_id: dto.media_connection_id.clone(),
                    })),
                )),
                _ => todo!(),
            });
        let repository: Box<dyn Repository> = Box::new(repository);

        // パラメータの生成
        let logger = helper::create_logger();
        let program_state = helper::create_program_state();
        let function = helper::create_functions();
        let param = RequestDto::Media(MediaRequestDto::Call {
            params: CallQueryDto {
                peer_id: PeerId::new("peer_id"),
                token: Token::try_create("pt-9749250e-d157-4f80-9ee2-359ce8524308").unwrap(),
                target_id: PeerId::new("target_id"),
                constraints: None,
                redirect_params: None,
            },
        });

        // 実行
        let connect = Call::default();
        let response = connect
            .execute(&repository, &program_state, &logger, &function, param)
            .await;

        // 評価
        assert_eq!(response.unwrap(), answer);
    }
}
