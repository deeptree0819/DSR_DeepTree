# 주제 2 — YOLO 기반 Vision Pick & Place

Intel RealSense 깊이 카메라 + YOLO 객체 검출로 **마우스 클릭 없이 자동으로 물체 위치를 인식**하여 Doosan M0609 로봇이 집어 옮기는 데모.
DSR 직접 제어 버전과 MoveIt2 기반 3가지 시나리오(단일 픽 / 정렬 / 박스 적재) 노드를 포함합니다.

---

## 목차

1. [시스템 구조](#1-시스템-구조)
2. [필수 라이브러리 설치](#2-필수-라이브러리-설치)
3. [하드웨어 / 캘리브레이션 준비](#3-하드웨어--캘리브레이션-준비)
4. [빌드](#4-빌드)
5. [실행](#5-실행)
6. [노드별 동작 설명](#6-노드별-동작-설명)
7. [좌표 변환 흐름](#7-좌표-변환-흐름)
8. [파라미터 튜닝](#8-파라미터-튜닝)
9. [키보드 조작](#9-키보드-조작)
10. [트러블슈팅](#10-트러블슈팅)

---

## 1. 시스템 구조

```
RealSense D-series                   Doosan M0609 + OnRobot RG2
   │  /camera/.../image_raw                       ▲
   │  /camera/.../aligned_depth_to_color/image_raw│
   │  /camera/.../camera_info                     │ DSR_ROBOT2 / MoveIt2
   ▼                                              │
┌──────────────────────────┐                      │
│  yolo_pick_node          │                      │
│   - YOLO 추론 (ultralytics) │   pick_and_place()  │
│   - pixel → camera 3D     │ ──────────────────► │
│   - hand-eye → base 3D    │                      │
│   - 그리퍼 Modbus TCP     │                      │
└──────────────────────────┘                      ▼
        ▲                                Tool Charger 192.168.1.1:502
        │ T_gripper2camera.npy
        │ (hand-eye calibration)
        └──────── config/T_gripper2camera.npy
```

### 노드 종류 (entry point)

| 실행 명령 | 파일 | 동작 |
|---|---|---|
| `yolo_pick` | `yolo_pick_node.py` | DSR 직접 API. 신뢰도 1위 객체 1개 픽 → home XY place |
| `yolo_pick_moveit` | `yolo_pick_moveit_node.py` | MoveIt2 버전. 신뢰도 1위 객체 1개 픽 → home XY place |
| `yolo_pick_sort_moveit` | `yolo_pick_sort_moveit_node.py` | block/gear 검출 → 1열로 정렬 적재 (block 3 + gear 2) |
| `yolo_pick_box_moveit` | `yolo_pick_box_moveit_node.py` | 박스 위치를 먼저 탐색 후 검출 객체를 박스 안에 적재 |

---

## 2. 필수 라이브러리 설치

### Python 패키지 (pip)

```bash
cd ~/DSR_DeepTree/task_2/yolo_pick_ws
pip install -r requirements.txt
```

| 패키지 | 용도 |
|---|---|
| `ultralytics` | YOLO 추론 (yolo11n.pt / 커스텀 best.pt) |
| `opencv-python` | 영상 표시 / 시각화 |
| `numpy`, `scipy` | 좌표 변환 (ZYZ Euler → 회전행렬) |
| `pymodbus==2.5.3` | OnRobot RG2 그리퍼 Modbus TCP |
| `pyrealsense2` | (선택) RealSense SDK 직접 접근 |

> **pymodbus 버전 주의:** `3.x`는 API 변경으로 동작하지 않습니다. 반드시 `2.5.3`.

### ROS 2 패키지 (apt)

```bash
sudo apt install \
  ros-humble-realsense2-camera \
  ros-humble-cv-bridge \
  ros-humble-moveit \
  ros-humble-moveit-py
```

---

## 3. 하드웨어 / 캘리브레이션 준비

### 3-1. 하드웨어 연결

| 항목 | 확인 |
|---|---|
| 로봇 전원 | TP 켜짐, 비상정지 해제 |
| 이더넷 | PC ↔ 로봇 컨트롤러 (`192.168.1.100`) |
| 그리퍼 | RG2 Modbus TCP `192.168.1.1:502` |
| RealSense | USB 3.0 포트 (파란색)에 연결 — USB 2.0이면 동시 스트림 불가 |

연결 확인:

```bash
# 로봇
ping 192.168.1.100
# 그리퍼
ping 192.168.1.1
# RealSense (lsusb -t에서 5000M 확인)
lsusb -t | grep -A1 RealSense
```

### 3-2. Hand-Eye 캘리브레이션 파일

Eye-in-hand 변환행렬 `T_gripper2camera.npy`(4x4 동차행렬, mm 단위 translation)가 다음 경로에 있어야 합니다.

```
yolo_pick_ws/src/yolo_pick_demo/yolo_pick_demo/T_gripper2camera.npy
```

빌드 시 `setup.py`가 `share/yolo_pick_demo/config/`에 자동 설치합니다.

> 캘리브레이션 절차는 `ros2_ws/src/doosan-robot2/dsr_practice/dsr_practice/Calibration_Tutorial/` 참조.

### 3-3. YOLO 가중치

본 저장소에 사전 학습된 커스텀 모델 `best.pt`가 포함되어 있습니다 (`task_2/yolo_pick_ws/best.pt`).
**block / box / gear 3개 클래스**(0=block, 1=box, 2=gear)로 학습된 가중치이며, 별도 다운로드/학습 없이 그대로 사용합니다.

> 클래스 ID는 `_config.py`의 `CLS_BLOCK / CLS_BOX / CLS_GEAR` 상수 참조.

| 파일 | 위치 | 용도 |
|---|---|---|
| `best.pt` | `task_2/yolo_pick_ws/best.pt` | 본 데모용 커스텀 모델 (bar / gear) |
| `yolo11n.pt` | `task_2/yolo_pick_ws/yolo11n.pt` | (참고) ultralytics 기본 COCO 80 클래스 |

`yolo_pick_node.py:70`의 `YOLO_MODEL_PATH`를 본인이 clone한 경로에 맞게 수정하세요.

```python
YOLO_MODEL_PATH = "/home/<user>/DSR_DeepTree/task_2/yolo_pick_ws/best.pt"
```

---

## 4. 빌드

```bash
cd ~/DSR_DeepTree/task_2/yolo_pick_ws

# DSR 의존성 먼저 소싱
source ~/ros2_ws/install/setup.bash

colcon build --packages-select yolo_pick_demo
source install/setup.bash
```

설치 결과 확인:

```bash
ls install/yolo_pick_demo/share/yolo_pick_demo/config/
# T_gripper2camera.npy
# moveit_py.yaml
```

---

## 5. 실행

### 터미널 구성

```
[터미널 1] DSR 드라이버 (+ MoveIt)
[터미널 2] RealSense 드라이버
[터미널 3] yolo_pick_demo 노드
```

### 5-1. 터미널 1 — DSR 드라이버

DSR 직접 API 노드(`yolo_pick`)용:

```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
    mode:=real model:=m0609 host:=192.168.1.100
```

MoveIt2 노드(`yolo_pick_moveit`, `yolo_pick_sort_moveit`, `yolo_pick_box_moveit`)용:

```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
    mode:=real model:=m0609 host:=192.168.1.100
```

### 5-2. 터미널 2 — RealSense

```bash
ros2 launch realsense2_camera rs_align_depth_launch.py \
    depth_module.depth_profile:=640x480x30 \
    rgb_camera.color_profile:=640x480x30 \
    initial_reset:=true \
    align_depth.enable:=true
```

토픽 확인:

```bash
ros2 topic list | grep camera
# /camera/camera/color/camera_info
# /camera/camera/color/image_raw
# /camera/camera/aligned_depth_to_color/image_raw
```

### 5-3. 터미널 3 — YOLO Pick 노드 실행

```bash
source ~/DSR_DeepTree/task_2/yolo_pick_ws/install/setup.bash

# 기본 (DSR 직접 API)
ros2 run yolo_pick_demo yolo_pick

# MoveIt2 — 단일 픽
ros2 run yolo_pick_demo yolo_pick_moveit

# MoveIt2 — block/gear 정렬 적재
ros2 run yolo_pick_demo yolo_pick_sort_moveit

# MoveIt2 — 박스 안 적재
ros2 run yolo_pick_demo yolo_pick_box_moveit
```

또는 launch:

```bash
ros2 launch yolo_pick_demo yolo_pick.launch.py model:=best.pt target_cls:=1
ros2 launch yolo_pick_demo yolo_pick_moveit.launch.py
ros2 launch yolo_pick_demo yolo_pick_sort_moveit.launch.py
ros2 launch yolo_pick_demo yolo_pick_box_moveit.launch.py
```

---

## 6. 노드별 동작 설명

### 6-1. `yolo_pick` (DSR 직접)

```
YOLO 검출 (전 프레임)
      ▼
신뢰도 1위 객체 선택
      ▼
중심 픽셀 → depth 조회 → 카메라 3D (X,Y,Z mm)
      ▼
T_gripper2camera, base→gripper 합성 → base 3D
      ▼
1) XY만 이동 (현재 z 유지)
2) z = base_z + Z_OFFSET 으로 하강
3) 그리퍼 close
4) SAFE_Z 까지 상승
5) home XY로 이동
6) place_z = max(PLACE_Z_FLOOR, z) 으로 하강
7) 그리퍼 open
8) SAFE_Z 복귀
```

### 6-2. `yolo_pick_moveit`

`yolo_pick`과 시퀀스 동일. DSR API 대신 MoveIt2 plan/execute 사용. RRTConnect 기본 플래너.

### 6-3. `yolo_pick_sort_moveit`

검출 객체를 클래스별로 분류해 **1열로 정렬 적재**.

| 클래스 | 정렬 기준 | Place 슬롯 |
|---|---|---|
| block | depth 오름차순 (가까운 것부터) | `BLOCK_X0=0.30 m`, `BLOCK_DX=0.05 m`, 3 슬롯 |
| gear  | bbox 크기 오름차순 (작은 것부터) | gap `0.10 m` 후 `GEAR_DX=0.08 m`, 2 슬롯 |

`PLACE_Y=0.27 m` 고정. 슬롯 카운터 리셋: `r` 키.

### 6-4. `yolo_pick_box_moveit`

```
[Phase 1] 박스 탐색
   home z + SCAN_Z_OFFSET(0.15 m)으로 상승
   → preview 1.5 s
   → YOLO로 box 클래스 검출
   → home 복귀 → box XY 위로 이동해 정밀 X,Y 측정
   → self.box_xyz 저장

[Phase 2] Pick & Place 루프
   block/gear 검출 → approach + 재검출 → pick
   → box XY 위 SAFE_Z에서 release
   → home 복귀
```

`s` 키로 박스 재탐색.

---

## 7. 좌표 변환 흐름

```
픽셀 (px, py)
   │
   │ depth_image[py, px]       (16UC1 mm 또는 32FC1 m)
   ▼
Z (mm)
   │ X = (px-ppx)*Z/fx
   │ Y = (py-ppy)*Z/fy
   ▼
카메라 좌표 (X, Y, Z) mm
   │
   │ T_base2gripper @ T_gripper2camera @ [X,Y,Z,1]ᵀ
   │   - T_base2gripper: posx → ZYZ Euler → 4x4
   │   - T_gripper2camera: hand-eye 캘리브 결과 (.npy)
   ▼
베이스 좌표 (Xb, Yb, Zb) mm
   │
   │ z_pick = Zb + Z_OFFSET   (TCP가 그리퍼 위쪽이므로 +)
   ▼
movel(target)
```

### Hand-eye 모델

본 코드는 **eye-in-hand** (카메라가 그리퍼에 부착) 가정입니다. `T_gripper2camera`는 그리퍼 좌표계 → 카메라 좌표계 변환행렬.

---

## 8. 파라미터 튜닝

### `yolo_pick_node.py` 상단 상수

```python
Z_OFFSET       = 220.0   # 검출 z에 더해 줄 오프셋 [mm] (TCP↔grasp point 거리)
SAFE_Z         = 400.0   # 안전 이송 높이 [mm]
PLACE_Z_FLOOR  = 250.0   # place 시 최소 z [mm]

YOLO_CONF_THRESH   = 0.5    # 검출 신뢰도 하한
YOLO_TARGET_CLS    = None   # None=전체, 정수=특정 클래스만
AUTO_PICK_INTERVAL = 3.0    # 자동 모드 픽 간격 [s]
```

### Launch 파라미터 (`yolo_pick.launch.py`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `model` | `yolo11n.pt` | YOLO 가중치 경로 |
| `conf_thresh` | `0.5` | 신뢰도 하한 |
| `target_cls` | `-1` | -1=전체, 정수=클래스 ID |
| `auto_interval` | `3.0` | 자동 픽 간격 [s] |

### MoveIt2 (`config/moveit_py.yaml`)

기본 플래너 `OMPL RRTConnect`, vel/acc scale `0.1`. 빠른 동작 필요 시 `ompl_rrtc` 또는 `pilz_lin` 그룹 참조.

### Sort 노드 슬롯 좌표

`yolo_pick_sort_moveit_node.py` 상단 상수로 트레이 위치에 맞게 조정.

```python
PLACE_Y         = 0.27   # 1열 y 좌표 [m]
BLOCK_X0        = 0.30   # block 첫 슬롯 x [m]
BLOCK_DX        = 0.05   # block 슬롯 간격 [m]
BLOCK_NUM_SLOTS = 3
BLOCK_GEAR_GAP  = 0.10
GEAR_DX         = 0.08
GEAR_NUM_SLOTS  = 2
```

---

## 9. 키보드 조작

| 키 | 동작 | 노드 |
|---|---|---|
| `p` | 1회 수동 픽 | 전체 |
| `a` | 자동 모드 토글 | 전체 |
| `r` | 슬롯 카운터 리셋 | sort |
| `s` | 박스 재탐색 | box |
| `ESC` | 종료 | 전체 |

---

## 10. 트러블슈팅

### `FileNotFoundError: T_gripper2camera.npy`

`setup.py`의 glob이 `.npy` 파일을 잘못된 경로에서 찾는 경우. `setup.py:16`을 다음으로 수정 후 재빌드.

```python
glob('config/*.yaml') + glob('yolo_pick_demo/*.npy')
```

### RealSense — `select() timeout` / `Frame didn't arrive within 5000`

```bash
# 1) 다른 프로세스가 카메라를 잡고 있는지
sudo fuser /dev/video2 /dev/video3 /dev/video4 /dev/video5 /dev/video6
# 모두 kill 후 재시도

# 2) USB 3.0 포트인지 확인 (5000M 표시)
lsusb -t | grep -A1 RealSense

# 3) 하드웨어 정상 여부
realsense-viewer
```

`cv2.VideoCapture(N)`로 RealSense에 직접 접근하면 select timeout이 자주 발생합니다. ROS realsense2_camera 또는 `pyrealsense2` SDK를 사용하세요.

### 로봇이 z 방향으로 반대로 움직임

- `Z_OFFSET` 부호 확인: 그리퍼가 TCP 아래로 늘어진 구조면 `+`, 반대 케이스면 `-`.
- Hand-eye 캘리브레이션 결과(`T_gripper2camera.npy`)의 회전 부분이 잘못 추정된 경우 base 좌표의 z가 반전될 수 있음 → 캘리브레이션 재수행.

### YOLO 검출 0개

```bash
# 로그에서 신뢰도 출력
ros2 run yolo_pick_demo yolo_pick
# YOLO_CONF_THRESH 낮춰보기 (0.3 등)
```

커스텀 모델인데 클래스 이름이 안 맞으면 `target_cls`도 -1로 바꿔 전체 클래스를 봐야 합니다.

### MoveIt 플래닝 실패

- vel/acc scale을 낮추세요 (`moveit_py.yaml`의 `max_velocity_scaling_factor` 0.05 등).
- 목표 포즈가 작업 영역 밖이거나 self-collision 시 발생 → base 좌표 로그 확인.

### `pymodbus` 그리퍼 통신 실패

```bash
pip install "pymodbus==2.5.3"
ping 192.168.1.1
```
