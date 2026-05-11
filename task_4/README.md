# Task 4: Isaac Sim ↔ MoveIt2 ↔ Real Robot Integration

Isaac Sim 시뮬레이션 환경에서 MoveIt2를 통해 실제 Doosan m0609 로봇팔과 RG2 그리퍼를 제어하는 통합 시스템입니다.

## 개요

이 프로젝트는 다음 3가지 모드를 지원합니다:

- **실물 연동 모드** (`mode:=real`): MoveIt2를 통해 실제 로봇 제어
- **Virtual 모드** (`mode:=virtual`): Virtual controller를 통한 시뮬레이션

시뮬레이션과 실물 로봇 간의 동기화는 다음과 같이 작동합니다:

```
Isaac Sim (물리 시뮬레이션)
    ↓ /isaac_joint_states
MoveIt2 (경로 계획 및 제어)
    ↓ /dsr_moveit_controller/joint_trajectory
    ↓ /rg2_gripper_controller/joint_trajectory
Real Robot (Doosan m0609 + RG2 Gripper)
```

## 주요 스크립트

### 1. `joint_state_passthrough.py`
- **역할**: Isaac Sim과 MoveIt2 간의 상태 동기화
- **기능**:
  - `/isaac_joint_states` 토픽 구독
  - `/joint_states` 토픽 발행 (MoveIt2 피드백)
  - 수동 모드 진입/종료 시 제어 전환
  - JSB(Joint State Broadcaster) 활성/비활성화 관리

### 2. `gripper_to_isaac.py`
- **역할**: MoveIt2의 그리퍼 명령을 Isaac Sim으로 전달
- **기능**:
  - GripperCommand 액션 서버 제공
  - Isaac Sim 그리퍼 상태 피드백
  - 손가락 위치 제어 및 모니터링

## 설치 및 실행

### 1. 워크스페이스 다운로드 및 빌드

```bash
# 레포지토리 클론
git clone https://github.com/YoonHJ97/DSR_DeepTree.git
cd DSR_DeepTree/task_4

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# Colcon Build
cd IsaacSim-ros_workspaces
colcon build 
cd ../doosan_ros2
colcon build
cd ../

# 환경 설정 파일 로드
source /IsaacSim-ros_workspaces/install/local_setup.bash
source /doosan_ros2/install/local_setup.bash
```

### 2. 실행 방법

#### 실물 로봇 연동 모드

```bash
# 터미널 1: Isaac Sim 실행
# Isaac Sim에서 scene 로드 및 Play 버튼 클릭

# 터미널 2: 실물 로봇 드라이버 실행
ros2 launch dsr_bringup dsr_bringup.launch.py

# 터미널 3: MoveIt2 + 동기화 노드 런칭
ros2 launch isaac_moveit moveit.launch.py mode:=real model:='m0609' host:=192.168.1.100
```

### 3. RViz2에서 MoveIt2 인터페이스 실행

MoveIt2 런칭 후 자동으로 RViz2가 열립니다:

```
RViz2 화면
├── Scene: Isaac Sim 시뮬레이션 상태 표시
├── Planning: 목표 위치 설정 및 경로 계획
├── Execution: "Plan and Execute" 버튼으로 실행
└── Gripper: GripperCommand 액션으로 제어
```

**RViz2 조작**:
- 로봇 목표 위치 설정: 마우스로 에드복스 드래그
- 경로 계획: `Plan` 버튼 클릭
- 실행: `Execute` 버튼 클릭 (또는 `Plan and Execute`)``

## 네트워크 토픽

### Isaac Sim → ROS2

| 토픽 | 타입 | 설명 |
|-----|------|------|
| `/isaac_joint_states` | `sensor_msgs/JointState` | 현재 관절 각도 (Isaac에서 발행) |
| `/isaac_joint_commands` | `sensor_msgs/JointState` | 목표 관절 각도 (ROS2에서 발행) |

### MoveIt2 ↔ 실물 로봇

| 토픽 | 타입 | 설명 |
|-----|------|------|
| `/dsr_moveit_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 로봇팔 제어 명령 |
| `/rg2_gripper_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 그리퍼 제어 명령 |
| `/joint_states` | `sensor_msgs/JointState` | 현재 로봇 상태 (피드백) |

## 문제 해결

### 1. "로봇이 이상한 방향으로 움직임"

`joint_state_passthrough.py`에서 부호 변환 확인:

```python
# 필요시 각 joint의 부호 수정
_JOINT_SIGN = {
    'joint_1': +1,  # -1로 변경하면 반대 방향
    'joint_2': +1,
    'joint_3': +1,
    'joint_4': +1,
    'joint_5': +1,
    'joint_6': +1,
    'rg2_finger_joint': +1,
}
```

### 2. "MoveIt2에서 경로 계획 실패"

```bash
# Isaac Sim 상태 확인
ros2 topic echo /isaac_joint_states

# MoveIt2 현재 상태 확인
ros2 topic echo /joint_states

# 두 값이 크게 차이나면 동기화 문제 → joint_state_passthrough.py 확인
```

## 참고사항

- **Isaac Sim Python 버전**: 3.11 (Bundle Python 사용)
- **ROS2 버전**: Humble
- **로봇**: Doosan m0609 (6-DOF 로봇팔)
- **그리퍼**: RG2 (병렬 그리퍼)
- **제어 루프**: 50Hz (passthrough echo rate)

## 라이선스

이 프로젝트는 NVIDIA Isaac Sim 샘플 코드 기반입니다.

## 문의

프로젝트 관련 문제는 GitHub Issues를 통해 보고해주세요.
