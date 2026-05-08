# TaskStatus.msg
# Pick & Place 작업의 현재 상태를 실시간으로 브로드캐스트

# ── 상태 상수 ─────────────────────────────
uint8 STATE_IDLE    = 0   # 대기 중
uint8 STATE_RUNNING = 1   # 작업 실행 중
uint8 STATE_SUCCESS = 2   # 완료
uint8 STATE_ERROR   = 3   # 오류

# ── 스텝 상수 ─────────────────────────────
string STEP_IDLE        = "IDLE"
string STEP_HOME        = "HOME"
string STEP_APPROACH    = "APPROACH"    # 접근 중 (z + offset)
string STEP_PICK        = "PICK"        # 기어 집기
string STEP_RETREAT     = "RETREAT"     # 픽 후 상승
string STEP_TRANSFER    = "TRANSFER"    # place 위치로 이동
string STEP_WIGGLE      = "WIGGLE"      # 마지막 기어 z축 흔들기
string STEP_PLACE       = "PLACE"       # 기어 놓기

# ── 필드 ──────────────────────────────────
std_msgs/Header header

uint8  state          # STATE_* 상수 사용
uint8  total_gears    # 전체 기어 수 (기본 4)
uint8  gear_index     # 현재 작업 중인 기어 번호 (1-based)
string current_step   # STEP_* 상수 사용
float32 progress_pct  # 전체 진행률 0.0 ~ 100.0 [%]
string  message       # 추가 설명 (오류 메시지 등)
