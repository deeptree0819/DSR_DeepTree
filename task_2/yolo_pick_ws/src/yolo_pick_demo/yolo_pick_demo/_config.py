"""공통 상수.

모든 MoveIt 기반 노드가 공유하는 로봇/그리퍼/카메라/YOLO 설정.
노드별 고유 상수(슬롯 위치, scan offset 등)는 각 노드 파일에 둔다.
"""

import math


# ── MoveIt ─────────────────────────────────────────
GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK    = "link_6"

HOME_JOINTS = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(90.0),
}

# ── 안전 작업 영역 (m, base_link) ────────────────────
SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.30
SAFE_Y_MAX =  0.30
SAFE_Z_MIN =  0.25

# ── Pick 파라미터 (m) ────────────────────────────────
Z_OFFSET = 0.20    # gripper tip ↔ link_6 (depth 측정 base z + 이 값 = pick_z)
SAFE_Z   = 0.40    # 안전 이동 높이

# ── Approach (재검출 직전 EE 미세 이동) ──────────────
APPROACH_OFFSET = (-0.05, -0.05)   # (dx, dy) m, Z 는 현재 유지
APPROACH_SETTLE = 0.5              # 이동 후 카메라 안정화 [s]

# ── 그리퍼 ──────────────────────────────────────────
GRIPPER_NAME     = "rg2"
TOOLCHARGER_IP   = "192.168.1.1"
TOOLCHARGER_PORT = 502

# ── YOLO ────────────────────────────────────────────
YOLO_MODEL_PATH    = "/home/ssu/yolo_pick_ws/best.pt"
YOLO_CONF_THRESH   = 0.5
AUTO_PICK_INTERVAL = 3.0    # 자동 모드 픽 간격 [s]

# ── 클래스 ID (v3 data.yaml: ['block', 'box', 'gear']) ──
CLS_BLOCK = 0
CLS_BOX   = 1
CLS_GEAR  = 2

# ── 카메라 토픽 ──────────────────────────────────────
TOPIC_CAM_INFO  = "/camera/camera/color/camera_info"
TOPIC_COLOR     = "/camera/camera/color/image_raw"
TOPIC_DEPTH     = "/camera/camera/aligned_depth_to_color/image_raw"
