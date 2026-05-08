# VoiceCommand.msg
# nlp_node 가 파싱한 명령을 stt_pick_and_place 노드로 전달
#
# 토픽: /robot_command

# ── 명령 종류 상수 ───────────────────────────────
uint8 CMD_UNKNOWN       = 0
uint8 CMD_HOME          = 1   # 홈 자세로 복귀
uint8 CMD_PICK          = 2   # 픽 동작만
uint8 CMD_PLACE         = 3   # 플레이스 동작만
uint8 CMD_PICKPLACE     = 4   # 픽 → 플레이스 연속
uint8 CMD_STOP          = 5   # 큐 비우기 / 동작 중단
uint8 CMD_GRIPPER_OPEN  = 6   # 그리퍼 열기
uint8 CMD_GRIPPER_CLOSE = 7   # 그리퍼 닫기
uint8 CMD_JOG           = 8   # 현재 위치 기준 방향 이동

# ── 방향 상수 (CMD_JOG 전용) ─────────────────────
uint8 DIR_NONE     = 0
uint8 DIR_FORWARD  = 1   # 앞  +x
uint8 DIR_BACKWARD = 2   # 뒤  -x
uint8 DIR_LEFT     = 3   # 왼  +y
uint8 DIR_RIGHT    = 4   # 오  -y
uint8 DIR_UP       = 5   # 위  +z
uint8 DIR_DOWN     = 6   # 아래 -z

# ── 필드 ────────────────────────────────────────
std_msgs/Header header

uint8  command          # CMD_* 상수
string raw_text         # STT 원본 텍스트 (디버그용)
string matched_keyword  # 매핑에 사용된 키워드
float32 confidence      # STT 인식 신뢰도 0.0 ~ 1.0
string  message         # 부가 설명

# ── 옵션 파라미터 ────────────────────────────────
uint8   task_id         # 기어 번호: 0=전체, 1~4=특정 기어
bool    use_wiggle      # 위글 동작 활성화 (기본 true)
uint8   direction       # DIR_* 상수 (CMD_JOG 전용)
