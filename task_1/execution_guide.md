# Pick & Place 데모 실행 절차 가이드

**대상 로봇:** Doosan M0609  
**그리퍼:** OnRobot RG2  
**ROS2 배포판:** Humble  
**패키지:** `pick_place_interfaces` / `pick_place_demo`

---

## 0. 실행 영상

[Google Drive 링크](https://drive.google.com/file/d/1g3YwA3QIpCv8fQWgglet9i-Or_jS1tvd/view?usp=sharing)

---

## 1. 사전 준비

### 1-0. 필수 라이브러리 설치

#### Python 패키지 (pip)

```bash
pip install pymodbus==2.5.3
```

| 패키지 | 용도 |
|--------|------|
| `pymodbus` | OnRobot RG2 그리퍼 Modbus TCP 통신 |

> **버전 주의:** `pymodbus 3.x`는 API가 변경되어 동작하지 않습니다. 반드시 `2.5.3`을 사용하세요.
>
> 설치 확인:
> ```bash
> python3 -c "import pymodbus; print(pymodbus.__version__)"
> # 2.5.3
> ```

---

### 1-1. 하드웨어 연결 확인

| 항목 | 확인 내용 |
|---|---|
| 로봇 전원 | TP(티치 펜던트) 화면 켜짐, 비상정지 해제 상태 |
| 이더넷 | PC ↔ 로봇 컨트롤러 직결 또는 같은 스위치 |
| 그리퍼 | Tool Changer에 RG2 체결, Modbus TCP IP `192.168.1.1:502` 통신 확인 |
| 작업 영역 | 기어 트레이 및 조립 지그 위치 고정 완료 |

### 1-2. 네트워크 설정

```bash
# PC 이더넷 인터페이스에 고정 IP 설정 (예: enp3s0)
sudo ip addr add 192.168.1.100/24 dev enp3s0

# 로봇 컨트롤러 통신 확인
ping 192.168.1.100      # 로봇 → PC
ping 192.168.1.1        # PC → Tool Changer (그리퍼)
```

### 1-3. 워크스페이스 구성

| 워크스페이스 | 경로 | 용도 |
|---|---|---|
| ROS2 base | `/opt/ros/humble/` | 자동 소싱 (`.bashrc`에 등록됨) |
| ros2_ws | `~/ros2_ws/` | DSR 드라이버 + MoveIt 설정 |
| ws_moveit | `~/ws_moveit/` | `moveit_configs_utils`, `launch_param_builder` (소스 빌드) |
| pick_place_ws | `~/pick_place_ws/` | pick_place_demo / pick_place_interfaces |

> **`.bashrc` 등록 별칭 (새 터미널마다 사용)**
> ```bash
> ros2_ws        # source ~/ros2_ws/install/local_setup.bash
> ws_moveit      # source ~/ws_moveit/install/local_setup.bash
> pick_place_ws  # source ~/pick_place_ws/install/local_setup.bash
> ```

---

## 2. 빌드

```bash
cd ~/pick_place_ws

# 인터페이스 패키지 먼저 빌드 (의존성 순서 필수)
colcon build --packages-select pick_place_interfaces
source install/local_setup.bash

# 노드 패키지 빌드
colcon build --packages-select pick_place_demo
source install/local_setup.bash
```

> **주의:** `source install/local_setup.bash`를 중간에 반드시 실행해야  
> `pick_place_demo`가 `pick_place_interfaces`의 메시지 타입을 찾을 수 있습니다.  
> 빌드 전 `ros2_ws` 별칭으로 ros2_ws를 먼저 소싱해야 DSR 의존성이 해결됩니다.

빌드 결과 확인:

```bash
ros2 interface list | grep pick_place
# pick_place_interfaces/action/PickAndPlace
# pick_place_interfaces/msg/GripperState
# pick_place_interfaces/msg/TaskStatus
# pick_place_interfaces/srv/GetTaskList
# pick_place_interfaces/srv/SetGripper
```

---

## 3. 실행

### 3-1. 터미널 구성 (3개)

```
[터미널 1] DSR 드라이버 + MoveIt   ← ros2_ws + ws_moveit 소싱 필요
[터미널 2] pick_place_server        ← ros2_ws + ws_moveit + pick_place_ws 소싱 필요
[터미널 3] pick_place_client        ← pick_place_ws 소싱 필요 (MoveIt 미사용)
```

> **주의:** `/opt/ros/humble/setup.bash`는 `.bashrc`에 이미 등록되어 있어  
> 새 터미널을 열면 자동으로 적용됩니다. 별도로 실행할 필요 없습니다.

---

### 3-2. 터미널 1 — DSR 드라이버 + MoveIt 실행

```bash
# 1) ros2_ws 소싱 (DSR 드라이버)
ros2_ws

# 2) ws_moveit 소싱 (moveit_configs_utils — DSR MoveIt launch 의존)
ws_moveit

# 3) 실제 로봇 연결 모드로 실행
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
    mode:=real \
    model:=m0609 \
    host:=192.168.1.100
```

MoveIt이 `/joint_states` 토픽을 수신하고  
`Planning Scene Monitor`가 초기화될 때까지 대기합니다 (약 10~15초).

### 3-3. 터미널 2 — pick_place_server 실행

```bash
# 1) ros2_ws 소싱 (DSR 메시지 타입 의존성)
ros2_ws

# 2) ws_moveit 소싱 (launch 파일이 moveit_configs_utils 사용)
ws_moveit

# 3) pick_place_ws 소싱
pick_place_ws

# 4) 서버 실행
ros2 launch pick_place_demo pick_place.launch.py
```

정상 시작 로그 확인:

```
[pick_place_server] RobotController 초기화 완료
[pick_place_server] PickPlaceServer 준비 완료 (/pick_and_place)
```

### 3-4. 터미널 3 — pick_place_client 실행

```bash
# 1) pick_place_ws 소싱 (pick_place_interfaces만 사용; MoveIt 미사용)
pick_place_ws

# 2) 클라이언트 실행
# 전체 4개 기어 순서대로 실행 (기본)
ros2 run pick_place_demo pick_place_client

# 특정 기어 1개만 실행
ros2 run pick_place_demo pick_place_client --id 2

# 위글 비활성화
ros2 run pick_place_demo pick_place_client --no-wiggle

# 접근 오프셋 변경 (기본 0.05 m)
ros2 run pick_place_demo pick_place_client --approach 0.08
```

---

## 4. 실시간 모니터링

### 4-1. 작업 상태 확인

```bash
# 로봇 작업 상태 (5 Hz 브로드캐스트)
ros2 topic echo /robot_status

# 그리퍼 상태 (10 Hz 브로드캐스트)
ros2 topic echo /gripper_state
```

### 4-2. 그리퍼 단독 제어 (서비스 직접 호출)

```bash
# 그리퍼 열기 (50 mm)
ros2 service call /set_gripper pick_place_interfaces/srv/SetGripper \
    "{width_mm: 50.0, force_n: 20.0, blocking: true}"

# 그리퍼 닫기 (20 mm)
ros2 service call /set_gripper pick_place_interfaces/srv/SetGripper \
    "{width_mm: 20.0, force_n: 20.0, blocking: true}"
```

### 4-3. 등록된 작업 목록 조회

```bash
ros2 service call /get_task_list pick_place_interfaces/srv/GetTaskList "{}"
```

### 4-4. 액션 수동 취소

클라이언트가 실행 중인 상태에서 별도 터미널로 취소합니다.

```bash
ros2 service call /pick_and_place/_action/cancel_goal action_msgs/srv/CancelGoal "{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, stamp: {sec: 0, nanosec: 0}}}"
```

취소되면 현재 스텝 완료 후 홈으로 복귀하고 클라이언트에 실패 결과가 반환됩니다.

```
[pick_place_client]: [실패] N개 기어 완료 후 중단 — 취소됨
```

> `uuid` 와 `stamp` 를 모두 0으로 보내면 실행 중인 **모든** 목표가 취소됩니다 (ROS2 액션 spec).

---

## 5. 실행 흐름 요약

기어 1개당 스텝 순서:

```
HOME → APPROACH(pick 위) → PICK(내려가서 집기) → RETREAT(pick 위로)
  → TRANSFER(place 위) → [WIGGLE] → PLACE(내려가서 놓기) → RETREAT(place 위)
  → 다음 기어 HOME ...
```

```
클라이언트                     서버 (robot_controller + pick_place_server)
    │                                         │
    │── Goal (use_all_tasks=true) ──────────>│
    │<── Feedback (HOME,      0%) ───────────│ 홈 이동
    │<── Feedback (APPROACH,  4%) ───────────│ 기어1 pick 접근
    │<── Feedback (PICK,      8%) ───────────│ 기어1 집기
    │<── Feedback (RETREAT,  12%) ───────────│ 기어1 pick 후퇴
    │<── Feedback (TRANSFER, 16%) ───────────│ 기어1 place 이송
    │<── Feedback (PLACE,    20%) ───────────│ 기어1 놓기 + 후퇴
    │         ...  (기어 2, 3 반복)           │
    │<── Feedback (WIGGLE,   96%) ───────────│ 기어4 위글
    │<── Feedback (PLACE,    99%) ───────────│ 기어4 놓기 + 후퇴
    │<── Result (success=true) ──────────────│ HOME 복귀
```

---

## 6. 종료

```bash
# 터미널 3: 클라이언트는 작업 완료 후 자동 종료

# 터미널 2: Ctrl+C
# 터미널 1: Ctrl+C
```

---

## 7. 자주 발생하는 오류

| 오류 메시지 | 원인 | 해결 |
|---|---|---|
| `Planning failed` | MoveIt 경로 계획 실패 | 장애물 확인, planning_time 늘리기 |
| `gripper command failed` | Modbus TCP 연결 끊김 | Tool Changer IP/포트 확인 |
| `/pick_and_place 서버 대기 중` 에서 멈춤 | 서버 미실행 | 터미널 2 상태 확인 |
| `z ... clamped to SAFE_Z_MIN` | 포즈 좌표가 안전 영역 하한 이하 | `GEAR_TASKS` 포즈 좌표 수정 |
| `joint_states` 수신 안 됨 | DSR 드라이버 미실행 | 터미널 1 먼저 확인 |
