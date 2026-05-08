# 로봇 안전 동작 범위 및 기본 파라미터

**대상:** Doosan M0609 + OnRobot RG2  
**기준 프레임:** `base_link`  
**코드 위치:** `pick_place_demo/robot_controller.py`

---

## 1. 안전 작업 영역 (Safe Workspace)

로봇이 이동할 수 있는 TCP 위치의 허용 범위입니다.  
`robot_controller.py`의 `_clamp_to_safe_workspace()`에서 자동으로 적용되며,  
범위를 벗어나는 목표 포즈는 경계값으로 클램핑되고 경고 로그가 출력됩니다.

```
base_link 기준 (단위: m)

        y
        │  SAFE_Y_MAX = +0.3
        │ ┌──────────────────┐
        │ │  허용 작업 영역   │
        │ │                  │
        │ │  x: 0.0 ~ (제한없음) ─── 로봇 전방
        │ │  z: 0.27 ~ (제한없음) ── 테이블면 위
        │ └──────────────────┘
        │  SAFE_Y_MIN = -0.3
        └──────────────────────── x
```

| 파라미터 | 값 | 설명 |
|---|---|---|
| `SAFE_X_MIN` | `0.0 m` | 로봇 후방 침범 방지 |
| `SAFE_Y_MIN` | `-0.3 m` | 우측 한계 |
| `SAFE_Y_MAX` | `+0.3 m` | 좌측 한계 |
| `SAFE_Z_MIN` | `0.27 m` | 테이블면 충돌 방지 |

> **변경 방법:** `robot_controller.py` 상단 상수를 수정하거나,  
> 추후 ROS2 파라미터(`declare_parameter`)로 런치 시 오버라이드할 수 있습니다.

---

## 2. HOME 자세

작업 시작/종료 및 오류 복구 시 복귀하는 기준 관절각입니다.

| 관절 | 각도 (deg) | 각도 (rad) |
|---|---|---|
| joint_1 | 0.0 | 0.000 |
| joint_2 | 0.0 | 0.000 |
| joint_3 | 90.0 | 1.571 |
| joint_4 | 0.0 | 0.000 |
| joint_5 | 90.0 | 1.571 |
| joint_6 | 0.0 | 0.000 |

```python
# robot_controller.py
HOME_JOINTS_RAD = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(0.0),
}
```

---

## 3. 모션 플래너 파라미터

### 3-1. HOME 이동 (OMPL RRTConnect)

경로 품질보다 빠른 계획이 필요한 joint-space 이동에 사용합니다.

| 파라미터 | 값 |
|---|---|
| `planning_pipeline` | `ompl` |
| `planner_id` | `RRTConnect` |
| `max_velocity_scaling_factor` | `0.2` (20%) |
| `max_acceleration_scaling_factor` | `0.1` (10%) |
| `planning_time` | `2.0 s` |

### 3-2. Pick & Place 이동 (Pilz PTP)

pick/place 각 스텝의 Cartesian 이동에 사용합니다.  
Pilz PTP는 시작-끝 점 사이를 직선에 가까운 경로로 계획해 예측 가능한 동작을 보장합니다.

| 파라미터 | 값 |
|---|---|
| `planning_pipeline` | `pilz_industrial_motion_planner` |
| `planner_id` | `PTP` |
| `max_velocity_scaling_factor` | `0.15` (15%) |
| `max_acceleration_scaling_factor` | `0.1` (10%) |
| `planning_time` | `2.0 s` |

> 속도 스케일(`max_velocity_scaling_factor`)은 클라이언트 Goal의  
> `max_velocity_scale` 필드로 런타임에 변경할 수 있습니다.

---

## 4. OnRobot RG2 그리퍼 파라미터

| 파라미터 | 값 | 설명 |
|---|---|---|
| `TOOLCHARGER_IP` | `192.168.1.1` | Tool Changer Modbus TCP IP |
| `TOOLCHARGER_PORT` | `502` | Modbus TCP 포트 |
| `GRIPPER_OPEN_WIDTH_MM` | `50.0 mm` | 기어 픽업 전/놓기 후 열림 폭 |
| `GRIPPER_CLOSE_WIDTH_MM` | `20.0 mm` | 기어 파지 폭 |
| `GRIPPER_FORCE_N` | `20.0 N` | 파지력 (RG2 최대: 40 N) |
| RG2 최대 폭 | `110.0 mm` | 알루미늄 핑거 기준 |
| RG2 최대 힘 | `40.0 N` | 초과 설정 시 드라이버가 거부 |

```python
# robot_controller.py
GRIPPER_OPEN_WIDTH_MM  = 50.0   # mm
GRIPPER_CLOSE_WIDTH_MM = 20.0   # mm
GRIPPER_FORCE_N        = 20.0   # N
```

---

## 5. 기어 Pick & Place 포즈 파라미터

모든 포즈는 `base_link` 기준 TCP 좌표이며, orientation은 툴이 수직으로 내려다보는 자세입니다  
`(qx=0, qy=1, qz=0, qw=0)`.

| 기어 | Pick 위치 (x, y, z) m | Place 위치 (x, y, z) m |
|---|---|---|
| Gear 1 | (0.403, +0.094, 0.280) | (0.398, -0.206, 0.280) |
| Gear 2 | (0.395, +0.197, 0.280) | (0.392, -0.101, 0.280) |
| Gear 3 | (0.490, +0.150, 0.280) | (0.486, -0.149, 0.280) |
| Gear 4 | (0.427, +0.148, 0.280) | (0.426, -0.153, 0.280) |

### 5-1. 접근/후퇴 오프셋

```python
APPROACH_OFFSET = 0.05   # m  (pick/place 포즈 z 축 상방 오프셋)
```

pick 또는 place 위치에 직접 내려가기 전, `z + 0.05 m` 위에서 먼저 접근하여  
충돌 없이 수직 하강합니다. 클라이언트 Goal의 `approach_offset_m`으로 변경 가능합니다.

### 5-2. 마지막 기어 Wiggle 파라미터

조립 공차 보정을 위해 마지막 기어(Gear 4)에만 적용됩니다.

| 파라미터 | 값 | 설명 |
|---|---|---|
| `WIGGLE_Z` | `0.295 m` | wiggle 수행 높이 (place_z보다 15 mm 위) |
| `WIGGLE_YAW_DEG` | `5.0 deg` | 좌우 회전 각도 |
| `WIGGLE_COUNT` | `3` | 좌우 반복 횟수 (총 6회 회전) |

클라이언트에서 `use_wiggle=false`를 전달하면 비활성화됩니다.

---

## 6. 파라미터 변경 가이드

### 포즈 좌표 수정

`robot_controller.py`의 `GEAR_TASKS` 리스트에서 직접 수정합니다.

```python
GEAR_TASKS = [
    {
        "pick":  {"pos": (x, y, z), "ori": (qx, qy, qz, qw)},
        "place": {"pos": (x, y, z), "ori": (qx, qy, qz, qw)},
    },
    ...
]
```

새 좌표를 입력한 뒤 반드시 재빌드합니다:

```bash
colcon build --packages-select pick_place_demo
source install/setup.bash
```

### 속도 스케일 런타임 변경

재빌드 없이 클라이언트 실행 시 적용합니다:

```bash
# 속도 20%로 실행
ros2 run pick_place_demo pick_place_client   # Goal.max_velocity_scale=0.15 기본
```

> 현재 클라이언트 코드에서 `goal.max_velocity_scale = 0.15`로 고정되어 있습니다.  
> CLI 인자로 받고 싶다면 `--velocity` 옵션을 `pick_place_client.py`에 추가하세요.
