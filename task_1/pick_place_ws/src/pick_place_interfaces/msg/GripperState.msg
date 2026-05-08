# GripperState.msg
# OnRobot RG2/RG6 그리퍼의 현재 상태를 실시간으로 브로드캐스트
# onrobot.py의 get_width(), get_status() 결과를 반영

std_msgs/Header header

# ── 그리퍼 폭 ──────────────────────────────
float32 current_width_mm   # 현재 폭 [mm] (read_holding_registers 267)
float32 target_width_mm    # 목표 폭 [mm]

# ── 힘 ─────────────────────────────────────
float32 current_force_n    # 현재 파지력 [N]
float32 target_force_n     # 목표 파지력 [N]

# ── 상태 플래그 (onrobot.py get_status() 매핑) ──
bool busy             # bit0: 모션 진행 중 → 이때 새 명령 무시됨
bool grip_detected    # bit1: 파지 감지됨
bool safety1_pushed   # bit2: 안전 스위치 1 눌림
bool safety1_trigged  # bit3: 안전 회로 1 활성화 (전원 재시작 필요)
bool safety2_pushed   # bit4: 안전 스위치 2 눌림
bool safety2_trigged  # bit5: 안전 회로 2 활성화 (전원 재시작 필요)
bool safety_error     # bit6: 전원 ON 시 안전 스위치 눌림
