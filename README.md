# aai4r_tm
AAI4R Task Manager for Restaurant Service 1

- Programmed by Minsu Jang (minsu@etri.re.kr)

## Topics

- TTS 요청: /aai4r/tts_req (std_msgs/String)
- TTS 응답: /aai4r/tts_res (std_msgsString)
- STT 응답: /aai4r/stt_res (std_msgsString)
- Expression 요청: /aai4r/expression_req (std_msgs/String)

## 실행 방법

1. Docker 컨테이너 생성

```dockerbuilt.sh```

2. Docker 컨테이너 실행

```docker run -it --rm aai4r/tm```

- 네트워크 등은 환경에 맞게 설정해 주세요.
- 명령을 실행하면 아래와 같이 프롬프트가 뜨고 숫자를 선택하면 메시지를 발행할 수 있습니다.
- 위 토픽 중 응답 토픽으로 메시지를 수신하면 수신한 메시지 내용을 화면에 출력합니다.


## 시스템 흐름

