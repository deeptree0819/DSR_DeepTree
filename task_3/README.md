# 주제 3 — STT 음성 명령 기반 로봇 Pick & Place

마이크 입력 → 음성 인식(STT) → 자연어 처리(NLP) → MoveIt2 동작 실행 → TTS 상태 안내  
4 개의 ROS 2 노드가 토픽/서비스로 연결되는 파이프라인 예제입니다.

---

## 목차

1. [시스템 구조](#1-시스템-구조)
2. [필수 라이브러리 설치](#2-필수-라이브러리-설치)
3. [빌드](#3-빌드)
4. [실행](#4-실행)
5. [음성 명령 레퍼런스](#5-음성-명령-레퍼런스)
6. [ROS 2 인터페이스](#6-ros-2-인터페이스)
7. [메시지 변환 로직 예제](#7-메시지-변환-로직-예제)
8. [행동 로직 예제](#8-행동-로직-예제)
9. [TTS 기반 작업 상태 안내 예제](#9-tts-기반-작업-상태-안내-예제)
10. [마이크 없이 테스트](#10-마이크-없이-테스트)
11. [파라미터 튜닝](#11-파라미터-튜닝)
12. [트러블슈팅](#12-트러블슈팅)

---

## 1. 시스템 구조

```
마이크
  │
  ▼
┌─────────────┐   /stt_result (String)   ┌─────────────┐
│  stt_node   │ ───────────────────────► │  nlp_node   │
└─────────────┘                          └──────┬──────┘
                                                │ /robot_command (VoiceCommand)
                                                ▼
                                    ┌───────────────────────┐
                                    │  stt_pick_and_place   │  ← MoveIt2 + OnRobot RG2
                                    └──────────┬────────────┘
                          /tts_input (String)  │  /robot_state (String)
                                    ┌──────────┘  /robot_status (String)
                                    ▼
                             ┌─────────────┐
                             │  tts_node   │ → 스피커
                             └─────────────┘
```

### 노드 역할

| 노드 | 파일 | 역할 |
|------|------|------|
| `stt_node` | `stt_node.py` | Google STT로 마이크 음성을 텍스트로 변환 |
| `nlp_node` | `nlp_node.py` | 텍스트 → VoiceCommand 메시지 변환 (키워드 매핑, 기어 번호·방향 파싱) |
| `stt_pick_and_place` | `stt_pick_and_place.py` | VoiceCommand 수신 → MoveIt2 + 그리퍼 실행 |
| `tts_node` | `tts_node.py` | `/tts_input` 구독 → gTTS + pygame으로 음성 출력 |

---

## 2. 필수 라이브러리 설치

### Python 패키지 (pip)

```bash
pip install \
  SpeechRecognition \
  gtts \
  pygame \
  pyaudio \
  pymodbus
```

> **pyaudio 설치 오류 시:**
> ```bash
> sudo apt install portaudio19-dev
> pip install pyaudio
> ```

---

## 3. 빌드

```bash
cd ~/stt_robot_ws

# 인터페이스 먼저 빌드 후 데모 빌드
colcon build --packages-select stt_robot_interfaces
source install/setup.bash
colcon build --packages-select stt_robot_demo
source install/setup.bash
```

---

## 4. 실행

### 기본 실행

```bash
ros2 launch stt_robot_demo stt_demo.launch.py
```

### 옵션 지정

```bash
ros2 launch stt_robot_demo stt_demo.launch.py \
  language:=ko-KR \
  tts_lang:=ko \
  mic_index:=0 \
  vel_scale:=0.15 \
  gripper_ip:=192.168.1.1 \
  use_gripper:=false
```

### 런치 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `language` | `ko-KR` | STT 인식 언어 (`en-US` 등) |
| `tts_lang` | `ko` | TTS 출력 언어 |
| `mic_index` | `-1` | 마이크 장치 인덱스 (-1: 시스템 기본) |
| `vel_scale` | `0.15` | 로봇 최대 속도 스케일 (0.0 ~ 1.0) |
| `gripper_ip` | `192.168.1.1` | OnRobot ToolCharger IP |
| `use_gripper` | `true` | `false` 시 그리퍼 없이 시뮬레이션 |

---

## 5. 음성 명령 레퍼런스

### 기본 동작

| 음성 예시 | 동작 |
|-----------|------|
| "홈", "복귀", "처음" | 홈 자세로 이동 |
| "정지", "멈춰", "그만" | 동작 취소 + 큐 비우기 |
| "그리퍼 열어", "열어" | 그리퍼 열기 |
| "그리퍼 닫아", "닫아" | 그리퍼 닫기 |

### Pick / Place (기어 번호 필수)

| 음성 예시 | 동작 |
|-----------|------|
| "기어 1번 픽" | 기어 1번 위치에서 파지 |
| "기어 2번 플레이스" | 기어 2번 위치에 내려놓기 (파지 상태에서만 가능) |
| "첫번째 기어 집어" | 기어 1번 픽 |
| "세번째 기어 놓아" | 기어 3번 플레이스 |

### Pick & Place 시퀀스

| 음성 예시 | 동작 |
|-----------|------|
| "조립", "실행" | 전체 4개 기어 순서대로 pickplace |
| "기어 1번 조립" | 기어 1번만 pickplace |
| "두번째 기어 실행" | 기어 2번만 pickplace |
| "위글없이 조립" | 위글 동작 없이 전체 실행 |
| "기어 3번 위글없이" | 기어 3번, 위글 없음 |

### 방향 이동 (Jog)

현재 End-Effector 위치 기준으로 `JOG_OFFSET`(기본 5 cm) 이동합니다.

| 음성 | 축 | 방향 |
|------|----|------|
| "앞으로", "앞", "전진" | X | +x |
| "뒤로", "뒤", "후진" | X | -x |
| "왼쪽", "왼" | Y | +y |
| "오른쪽", "오른" | Y | -y |
| "위로", "위" | Z | +z |
| "아래로", "아래" | Z | -z |

> Jog 스텝 크기 변경: `stt_pick_and_place.py` 상단의 `JOG_OFFSET = 0.05` 수정

---

## 6. ROS 2 인터페이스

### 토픽

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/stt_result` | `std_msgs/String` | stt_node → nlp_node | STT 인식 텍스트 |
| `/robot_command` | `VoiceCommand` | nlp_node → stt_pick_and_place | 파싱된 명령 |
| `/tts_input` | `std_msgs/String` | 여러 노드 → tts_node | TTS 출력 요청 |
| `/robot_state` | `std_msgs/String` | stt_pick_and_place → 외부 | 로봇 현재 상태 (2 Hz) |
| `/robot_status` | `std_msgs/String` | stt_pick_and_place → 외부 | 동작 결과 메시지 |

### 서비스

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/speak` | `Speak` | TTS 직접 요청 (blocking 옵션) |

### VoiceCommand.msg

```
# 명령 상수
uint8 CMD_UNKNOWN       = 0
uint8 CMD_HOME          = 1
uint8 CMD_PICK          = 2
uint8 CMD_PLACE         = 3
uint8 CMD_PICKPLACE     = 4
uint8 CMD_STOP          = 5
uint8 CMD_GRIPPER_OPEN  = 6
uint8 CMD_GRIPPER_CLOSE = 7
uint8 CMD_JOG           = 8

# 방향 상수 (CMD_JOG 전용)
uint8 DIR_NONE     = 0
uint8 DIR_FORWARD  = 1   # 앞 +x
uint8 DIR_BACKWARD = 2   # 뒤 -x
uint8 DIR_LEFT     = 3   # 왼 +y
uint8 DIR_RIGHT    = 4   # 오 -y
uint8 DIR_UP       = 5   # 위 +z
uint8 DIR_DOWN     = 6   # 아래 -z

# 필드
std_msgs/Header header
uint8   command
string  raw_text
string  matched_keyword
float32 confidence
string  message
uint8   task_id     # 0=전체, 1~4=특정 기어
bool    use_wiggle
uint8   direction
```

### /robot_state 포맷

```
IDLE
HOMING
PICKING(1)
HOLDING(2)  held_gear=2
PLACING(2)
JOGGING
ERROR
```

---

## 7. 메시지 변환 로직 예제

`nlp_node.py`는 STT 텍스트를 `VoiceCommand`로 변환합니다.

### 키워드 매핑 흐름

```
STT 텍스트: "기어 2번 위글없이 조립"
      │
      ├─ _parse_jog()      → DIR_NONE  (방향 키워드 없음)
      ├─ _parse_command()  → CMD_PICKPLACE  ("조립" 키워드 매칭)
      ├─ _parse_task_id()  → task_id = 2   ("기어 2번" 정규식)
      └─ _parse_use_wiggle() → False       ("위글없이" 감지)
      │
      ▼
VoiceCommand(command=4, task_id=2, use_wiggle=False)
```

### 방향 이동 우선순위

방향 키워드가 감지되면 다른 명령 파싱을 건너뜁니다.

```
STT 텍스트: "앞으로"
      │
      ├─ _parse_jog()  → DIR_FORWARD  ← 여기서 확정
      │
      ▼
VoiceCommand(command=CMD_JOG, direction=DIR_FORWARD)
```

### keyword_map.yaml 커스터마이징

`config/keyword_map.yaml`을 수정하면 **재빌드 없이** 키워드를 추가할 수 있습니다.

```yaml
keyword_map:
  pickplace:
    - "조립"
    - "실행"
    - "내가원하는키워드"   # ← 추가
```

---

## 8. 행동 로직 예제

### Pick 단독 시퀀스

```
기어 N번 픽 명령 수신
      │
      ├─ [검증] task_id == 0 → "기어 번호를 말해주세요" 후 종료
      ├─ [검증] holding == True → "이미 파지 중" 후 종료
      │
      ▼
approach (pick 위 +5cm)
      ↓
pick 위치로 하강
      ↓
그리퍼 CLOSE  →  _holding = True
      ↓
retreat (pick 위 +5cm으로 상승)
```

### Place 단독 시퀀스

```
기어 N번 플레이스 명령 수신
      │
      ├─ [검증] holding == False → "먼저 픽을 해주세요" 후 종료
      ├─ [검증] task_id == 0 → "기어 번호를 말해주세요" 후 종료
      │
      ▼
approach (place 위 +5cm)
      ↓
place 위치로 하강
      ↓
그리퍼 OPEN  →  _holding = False
      ↓
retreat (place 위 +5cm으로 상승)
```

### PickPlace 전체 시퀀스

```
기어 N번 (또는 전체 1~4번) pickplace 명령 수신
      │
      ▼  각 기어마다 반복
HOME → APPROACH(pick) → PICK → RETREAT → TRANSFER(place 위)
      → [WIGGLE: 마지막 기어 + use_wiggle=True]
      → PLACE → PLACE_RETREAT
      │
      ▼
HOME (완료 후 홈 복귀)
```

### 기어 작업 좌표 (base_link 기준)

| 기어 | Pick 위치 (x, y, z) | Place 위치 (x, y, z) |
|------|---------------------|----------------------|
| 1 | (0.398, 0.096, 0.280) | (0.398, -0.206, 0.280) |
| 2 | (0.392, 0.200, 0.280) | (0.392, -0.101, 0.280) |
| 3 | (0.486, 0.153, 0.280) | (0.486, -0.149, 0.280) |
| 4 | (0.427, 0.148, 0.280) | (0.426, -0.153, 0.280) |

좌표 변경: `stt_pick_and_place.py`의 `GEAR_TASKS` 리스트를 수정합니다.

### 안전 작업 영역 (자동 클램핑)

```python
SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.3
SAFE_Y_MAX =  0.3
SAFE_Z_MIN =  0.27
```

범위를 벗어난 목표는 자동으로 경계값으로 클램핑되며 경고 로그가 출력됩니다.

---

## 9. TTS 기반 작업 상태 안내 예제

`tts_node.py`는 `/tts_input` 토픽을 구독해 gTTS + pygame으로 음성을 출력합니다.

### 상태별 TTS 발화 시점

| 상황 | TTS 발화 주체 | 발화 내용 |
|------|--------------|-----------|
| 정지 명령 수신 | nlp_node | "동작을 중지합니다." |
| 홈 이동 시작 | stt_pick_and_place | "홈 자세로 이동합니다." |
| 기어 N번 픽 시작 | stt_pick_and_place | "기어 N번 픽을 시작합니다." |
| 기어 N번 플레이스 시작 | stt_pick_and_place | "기어 N번 플레이스를 시작합니다." |
| pickplace 시작 | stt_pick_and_place | "기어 N번 픽 앤 플레이스를 시작합니다." |
| 동작 완료 | stt_pick_and_place | "PICK 완료." / "PICKPLACE 완료." 등 |
| 동작 실패 (모션 오류) | stt_pick_and_place | "PICK 실패." |
| 기어 번호 미지정 | stt_pick_and_place | "기어 번호를 말해주세요." |
| 파지 없이 플레이스 | stt_pick_and_place | "먼저 픽을 해주세요." |
| 이미 파지 중 | stt_pick_and_place | "이미 기어 N번을 파지 중입니다." |

### /speak 서비스로 직접 TTS 요청

```bash
ros2 service call /speak stt_robot_interfaces/srv/Speak \
  "{text: '테스트 메시지입니다', rate: 1.0, blocking: false}"
```

### 토픽으로 TTS 요청

```bash
ros2 topic pub --once /tts_input std_msgs/msg/String "data: '안녕하세요'"
```

### TTS 설정 변경 (launch 파라미터)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `tts_lang` | `ko` | 출력 언어 (`en`, `ja` 등) |

---

## 10. 마이크 없이 테스트

### STT 우회 — 텍스트 직접 입력

```bash
# nlp → stt_pick_and_place 파이프라인 전체 테스트
ros2 topic pub --once /stt_result std_msgs/msg/String "data: '기어 1번 조립'"
ros2 topic pub --once /stt_result std_msgs/msg/String "data: '앞으로'"
ros2 topic pub --once /stt_result std_msgs/msg/String "data: '기어 2번 픽'"
ros2 topic pub --once /stt_result std_msgs/msg/String "data: '기어 2번 플레이스'"
ros2 topic pub --once /stt_result std_msgs/msg/String "data: '홈'"
ros2 topic pub --once /stt_result std_msgs/msg/String "data: '정지'"
```

### NLP 우회 — VoiceCommand 직접 발행

```bash
# 기어 1번 pickplace (use_wiggle=true)
ros2 topic pub --once /robot_command stt_robot_interfaces/msg/VoiceCommand \
  "{command: 4, task_id: 1, use_wiggle: true, message: 'PICKPLACE'}"

# 앞으로 jog
ros2 topic pub --once /robot_command stt_robot_interfaces/msg/VoiceCommand \
  "{command: 8, direction: 1, message: 'JOG'}"
```

### 상태 모니터링

```bash
# 터미널 A: 로봇 상태 확인
ros2 topic echo /robot_state

# 터미널 B: TTS 입력 확인
ros2 topic echo /tts_input

# 터미널 C: 파싱된 명령 확인
ros2 topic echo /robot_command
```

### 마이크 인덱스 확인

```bash
python3 -c "
import speech_recognition as sr
for i, name in enumerate(sr.Microphone.list_microphone_names()):
    print(i, name)
"
```

---

## 11. 파라미터 튜닝

### stt_pick_and_place.py 상단 상수

```python
JOG_OFFSET       = 0.05   # 방향 이동 한 스텝 [m]
APPROACH_OFFSET  = 0.05   # 픽/플레이스 접근 z 오프셋 [m]
WIGGLE_YAW_DEG   = 5.0    # 위글 yaw 각도 [deg]
WIGGLE_COUNT     = 3      # 위글 반복 횟수
WIGGLE_Z         = 0.295  # 위글 시 z 높이 [m]

GRIPPER_OPEN_WIDTH_MM  = 50.0  # 그리퍼 열림 너비 [mm]
GRIPPER_CLOSE_WIDTH_MM = 20.0  # 그리퍼 닫힘 너비 [mm]
GRIPPER_FORCE_N        = 20.0  # 그리퍼 힘 [N]
```

### stt_node 파라미터 (launch에서 조정)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `energy_threshold` | 300 | 발화 감지 민감도 (낮을수록 민감) |
| `pause_threshold` | 0.8 | 발화 종료 판단 침묵 시간 [s] |
| `phrase_timeout` | 3.0 | 최대 발화 인식 시간 [s] |

---

## 12. 트러블슈팅

### gTTS / pygame 설치 후 음성 출력 안 됨

```bash
# 오디오 장치 확인
aplay -l

# DISPLAY 환경변수 (헤드리스 환경)
export SDL_AUDIODRIVER=alsa
```

### pymodbus 버전 오류 (그리퍼)

```bash
pip install "pymodbus==2.5.3"
```

### MoveIt 플래닝 실패

- `vel_scale` 값을 낮춰보세요 (예: `0.05`)
- Pilz PTP 플래너가 도달 불가 포즈를 요청하면 OMPL로 자동 폴백되지 않으므로 좌표를 확인하세요

### STT 인식률 저하

- `energy_threshold`를 현재 환경 소음에 맞게 조정하세요
- 조용한 환경: 200~300 / 시끄러운 환경: 400~600

### nlp_node — 명령 미인식

```bash
# 어떤 텍스트가 들어오는지 확인
ros2 topic echo /stt_result

# keyword_map.yaml에 해당 키워드 추가 후 재실행 (재빌드 불필요)
```
