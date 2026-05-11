# DSR_DeepTree

Doosan M0609 + OnRobot RG2 기반 ROS 2 / MoveIt2 데모 모음 — Pick & Place, Vision (YOLO), STT 음성 제어, Isaac Sim 연동.

---

## 실행 영상

| 주제 | 영상 |
|---|---|
| Task 1 — Pick & Place 데모 | [Google Drive](https://drive.google.com/file/d/1g3YwA3QIpCv8fQWgglet9i-Or_jS1tvd/view?usp=sharing) |
| Task 2 — YOLO Vision (box 적재) | [Google Drive](https://drive.google.com/file/d/1dsnGKbeRjFXHxmgMH4i9QsRSVUQ3AwNT/view?usp=sharing) |
| Task 2 — YOLO Vision (sort 정렬) | [Google Drive](https://drive.google.com/file/d/1jSV0tIpiSE7unwCoQB0rZ__a4fXgRwPO/view?usp=sharing) |
| Task 3 — STT 음성 명령 Pick & Place | [Google Drive](https://drive.google.com/file/d/1cT0eEe5FO22uDxywymZdJGkxNznPfb52/view?usp=sharing) |

---

## 주제별 구성

### [Task 1 — Pick & Place 기본 데모](task_1/execution_guide.md)

Doosan M0609 + OnRobot RG2를 ROS 2 액션/서비스로 제어하는 Pick & Place 파이프라인.

- 패키지: `pick_place_demo`, `pick_place_interfaces`
- 인터페이스: `PickAndPlace.action`, `SetGripper.srv`, `GetTaskList.srv`
- 문서: [`task_1/execution_guide.md`](task_1/execution_guide.md), [`task_1/safety_and_parameters.md`](task_1/safety_and_parameters.md)

### [Task 2 — YOLO 기반 Vision Pick & Place](task_2/README.md)

Intel RealSense 깊이 카메라 + YOLO 객체 검출로 자동 Pick & Place. DSR 직접 제어 / MoveIt2 / 정렬 / 박스 적재 4가지 노드 제공.

- 패키지: `yolo_pick_demo`
- 노드: `yolo_pick`, `yolo_pick_moveit`, `yolo_pick_sort_moveit`, `yolo_pick_box_moveit`
- 핵심: Hand-eye calibration (`T_gripper2camera.npy`), pixel→camera→base 좌표 변환, OnRobot RG2 Modbus TCP 제어
- 문서: [`task_2/README.md`](task_2/README.md)

### [Task 3 — STT 음성 명령 Pick & Place](task_3/README.md)

마이크 → STT → NLP → MoveIt2 실행 → TTS 안내. 4개 ROS 2 노드로 구성된 음성 파이프라인.

- 패키지: `stt_robot_ws`
- 흐름: `stt_node` → `nlp_node` → `motion_node` → `tts_node`
- 문서: [`task_3/README.md`](task_3/README.md)

### [Task 4 — Isaac Sim 연동](task_4/README.md)

Isaac Sim에서 Doosan M0609 + OnRobot RG2 시뮬레이션과 ROS 2 브리지.

- `m0609_urdf/` — Doosan M0609 URDF / USD 자산
- `onrobot_rg2/` — OnRobot RG2 그리퍼 자산
- `doosan_ros2/` — Doosan ROS 2 드라이버 / 인터페이스
- `IsaacSim-ros_workspaces/` — Isaac Sim용 ROS 2 워크스페이스 빌드 스크립트
- `isaac_moveit_m0609_rg2.py` — Isaac Sim ↔ MoveIt2 통합 실행 스크립트
- 문서: [`task_4/READMD.md`](task_4/README.md)

---

## 공통 환경

| 항목 | 버전 / 사양 |
|---|---|
| 로봇 | Doosan M0609 |
| 그리퍼 | OnRobot RG2 (Modbus TCP, `192.168.1.1:502`) |
| 카메라 (Task 2) | Intel RealSense D-series |
| OS | Ubuntu 22.04 |
| ROS 배포판 | Humble |
| MoveIt | MoveIt 2 |
| Python | 3.10+ |

### 네트워크 기본값

```
PC ──── 192.168.1.x ──── Doosan 컨트롤러 (192.168.1.100)
                    └─── OnRobot RG2     (192.168.1.1:502)
```

---

## 빠른 시작

```bash
# 저장소 클론
git clone https://github.com/deeptree0819/DSR_DeepTree.git
cd DSR_DeepTree

# DSR ROS 2 드라이버 의존성은 별도 설치 필요
# (각 task 문서의 "필수 라이브러리 설치" 절 참조)
```

각 주제별 빌드/실행은 해당 디렉터리 README/가이드 참조.

---

## 라이선스

[LICENSE](LICENSE) 참조.
