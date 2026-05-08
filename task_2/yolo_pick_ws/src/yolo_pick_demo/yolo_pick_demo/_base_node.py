"""MoveIt 기반 Pick 노드 베이스 클래스.

공통 기능:
  - MoveIt 초기화, plan 파라미터
  - RealSense 카메라 콜백 (color / depth / intrinsics)
  - YOLO 모델 로드
  - Hand-Eye 변환, pixel→base 좌표 변환
  - Home 이동 + home_xyz/home_ori 캐싱
  - Approach + 재검출 루틴
  - cv2 메인 루프 (freeze 화면, 키 입력, 자동 모드)

자식 노드는 주로 다음을 override / 구현:
  - detect_and_pick(frame)        — pick 시퀀스
  - _select_target(detections)    — 다음 픽 대상 선정
  - _draw_detections(frame)       — (optional) 시각화
  - on_ready()                    — Home 이후 추가 init (e.g. scan)
  - is_auto_ready()               — auto 모드 트리거 가능 조건
  - _handle_key_extra(key)        — 추가 키 (e.g. 's' for box scan)
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from scipy.spatial.transform import Rotation

from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters

from .onrobot import RG
from . import _config as cfg
from ._motion import get_ee_matrix, make_pose, plan_and_execute
from . import _perception as perc

try:
    from ultralytics import YOLO
except ImportError as e:
    raise ImportError("pip install ultralytics") from e


class BaseMoveItPickNode(Node):
    """MoveIt + RealSense + YOLO + RG2 그리퍼 통합 베이스."""

    NODE_NAME        = "yolo_pick_base"
    MOVEIT_NODE_NAME = "yolo_pick_base_py"
    WINDOW_NAME      = "YOLO Pick"

    def __init__(self):
        super().__init__(self.NODE_NAME)
        log = self.get_logger()

        # ── 카메라 상태 ──
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.intrinsics  = None

        # ── 픽 상태 ──
        self.picking = False
        self.home_xyz = None     # (x, y, z) [m] — initialize_home 에서 설정
        self.home_ori = None     # quat dict {x, y, z, w}
        self._auto_mode = False
        self._last_pick_time = 0.0
        self._detections: list[dict] = []
        self._frozen_frame = None

        # ── Hand-Eye ──
        self.gripper2cam, calib_file = perc.load_hand_eye()
        log.info(f"Hand-Eye 로드: {calib_file}")

        # ── 그리퍼 ──
        self.gripper = RG(cfg.GRIPPER_NAME, cfg.TOOLCHARGER_IP, cfg.TOOLCHARGER_PORT)

        # ── MoveIt ──
        log.info("MoveItPy 초기화 중...")
        self.robot       = MoveItPy(node_name=self.MOVEIT_NODE_NAME)
        self.arm         = self.robot.get_planning_component(cfg.GROUP_NAME)
        self.robot_model = self.robot.get_robot_model()
        log.info("MoveItPy 초기화 완료")

        self.ompl_params = self._make_plan_params(
            "ompl", "RRTConnect", vel=0.2, acc=0.1, time=2.0)
        self.pilz_params = self._make_plan_params(
            "pilz_industrial_motion_planner", "PTP", vel=0.15, acc=0.1, time=2.0)

        # ── YOLO ──
        log.info(f"YOLO 모델 로드: {cfg.YOLO_MODEL_PATH}")
        self.yolo = YOLO(cfg.YOLO_MODEL_PATH)
        log.info("YOLO 모델 로드 완료")

        # ── 카메라 구독 ──
        self.create_subscription(CameraInfo, cfg.TOPIC_CAM_INFO,
                                 self._cam_info_cb, 10)
        self.create_subscription(Image, cfg.TOPIC_COLOR,
                                 self._color_cb, 10)
        self.create_subscription(Image, cfg.TOPIC_DEPTH,
                                 self._depth_cb, 10)

    # ════════════════════════════════════════════
    #  내부 헬퍼
    # ════════════════════════════════════════════
    def _make_plan_params(self, pipeline, planner_id, *,
                          vel: float, acc: float, time: float):
        p = PlanRequestParameters(self.robot)
        p.planning_pipeline = pipeline
        p.planner_id = planner_id
        p.max_velocity_scaling_factor = vel
        p.max_acceleration_scaling_factor = acc
        p.planning_time = time
        return p

    # ── 콜백 ──
    def _cam_info_cb(self, msg):
        self.intrinsics = {
            "fx": msg.k[0], "fy": msg.k[4],
            "ppx": msg.k[2], "ppy": msg.k[5],
        }

    def _color_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # ════════════════════════════════════════════
    #  Perception 래퍼
    # ════════════════════════════════════════════
    def transform_to_base(self, cam_xyz_m):
        return perc.transform_to_base(self.robot, self.gripper2cam, cam_xyz_m)

    def pixel_to_base(self, px, py):
        return perc.pixel_to_base(
            self.robot, self.gripper2cam,
            self.depth_image, self.intrinsics,
            px, py, self.get_logger())

    def run_yolo(self, frame):
        return perc.run_yolo(self.yolo, frame, self.depth_image)

    # ════════════════════════════════════════════
    #  Motion 래퍼
    # ════════════════════════════════════════════
    def plan_pose(self, x, y, z, ori, params=None) -> bool:
        return plan_and_execute(
            self.robot, self.arm, self.get_logger(),
            pose_goal=make_pose(x, y, z, ori),
            params=params or self.pilz_params)

    def plan_state(self, state, params=None) -> bool:
        return plan_and_execute(
            self.robot, self.arm, self.get_logger(),
            state_goal=state,
            params=params or self.ompl_params)

    def go_home_pose(self) -> bool:
        """관절 home 자세로 이동."""
        home_state = RobotState(self.robot_model)
        home_state.joint_positions = cfg.HOME_JOINTS
        home_state.update()
        return self.plan_state(home_state)

    # ════════════════════════════════════════════
    #  Approach + 재검출
    # ════════════════════════════════════════════
    def approach_and_redetect(self, target_cls_id: int, target_xy):
        """target XY 위로 EE 미세 이동 → 재검출 → 화면 중앙 가장 가까운 동일 클래스 detection.

        실패 시 None.
        """
        log = self.get_logger()
        ori = self.home_ori
        ox, oy = cfg.APPROACH_OFFSET
        cur_ee = get_ee_matrix(self.robot)
        ax = target_xy[0] + ox
        ay = target_xy[1] + oy
        az = cur_ee[2, 3]

        log.info(
            f"[Approach] target_xy=({target_xy[0]:.3f}, {target_xy[1]:.3f}) "
            f"+ offset -> EE=({ax:.3f}, {ay:.3f}, {az:.3f})"
        )
        if not self.plan_pose(ax, ay, az, ori):
            log.error("Approach 실패")
            return None
        time.sleep(cfg.APPROACH_SETTLE)

        if self.color_image is None:
            log.error("재검출 프레임 없음")
            return None
        new_frame = self.color_image.copy()
        new_detections = self.run_yolo(new_frame)
        self._detections   = new_detections
        self._frozen_frame = new_frame.copy()

        same_cls = [d for d in new_detections if d["cls_id"] == target_cls_id]
        if not same_cls:
            log.error(f"재검출 실패: cls={target_cls_id} 없음")
            return None

        h, w = new_frame.shape[:2]
        cx_img, cy_img = w // 2, h // 2
        return min(
            same_cls,
            key=lambda d: (d["cx"] - cx_img) ** 2 + (d["cy"] - cy_img) ** 2,
        )

    # ════════════════════════════════════════════
    #  시각화 (기본 구현 — 자식이 override 가능)
    # ════════════════════════════════════════════
    def _draw_detections(self, frame: np.ndarray) -> np.ndarray:
        """기본: 모든 detection 박스 + 다음 픽 대상은 녹색."""
        vis = frame.copy()
        next_target = (self._select_target(self._detections)
                       if self._detections else None)
        for det in self._detections:
            x1, y1, x2, y2 = det["box"]
            color = (0, 255, 0) if det is next_target else (255, 100, 0)
            label = f"{det['cls_name']} {det['conf']:.2f}"
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cv2.putText(vis, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            cv2.drawMarker(vis, (det["cx"], det["cy"]), color,
                           cv2.MARKER_CROSS, 20, 2)
        self._draw_hud(vis)
        return vis

    def _draw_hud(self, vis: np.ndarray):
        """상단 HUD (mode, detections 수)."""
        mode_txt = "AUTO" if self._auto_mode else "MANUAL"
        mode_col = (0, 255, 255) if self._auto_mode else (200, 200, 200)
        cv2.putText(vis, f"[{mode_txt}] {self._key_help_str()}",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_col, 2)
        cv2.putText(vis, f"detections: {len(self._detections)}",
                    (10, 52), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (180, 180, 180), 1)

    def _key_help_str(self) -> str:
        return "p:pick a:auto ESC:quit"

    # ════════════════════════════════════════════
    #  Pick 백그라운드 + freeze
    # ════════════════════════════════════════════
    def _pick_in_thread(self, frame: np.ndarray):
        if self.picking:
            return
        self._frozen_frame = frame.copy()

        def _work():
            try:
                self.detect_and_pick(frame)
            finally:
                self._frozen_frame = None

        threading.Thread(target=_work, daemon=True).start()

    # ════════════════════════════════════════════
    #  자식이 구현 / override 할 메서드 (hooks)
    # ════════════════════════════════════════════
    def detect_and_pick(self, frame: np.ndarray):
        raise NotImplementedError

    def _select_target(self, detections):
        raise NotImplementedError

    def on_ready(self):
        """Home 이동 완료 후 호출. 자식이 추가 init 가능 (e.g. scan_box)."""
        pass

    def is_auto_ready(self) -> bool:
        """auto 모드 트리거 전제 조건."""
        return True

    def _handle_key_extra(self, key: int):
        """ESC, p, a 외 추가 키 처리. 자식 override (e.g. 's' for scan)."""
        pass

    # ════════════════════════════════════════════
    #  메인 루프
    # ════════════════════════════════════════════
    def initialize_home(self) -> bool:
        log = self.get_logger()
        log.info("[Init] Home 이동")
        if not self.go_home_pose():
            log.error("Home 실패")
            return False
        time.sleep(0.5)

        T = get_ee_matrix(self.robot)
        self.home_xyz = (T[0, 3], T[1, 3], T[2, 3])
        qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
        self.home_ori = {"x": float(qx), "y": float(qy),
                         "z": float(qz), "w": float(qw)}
        log.info(f"[Init] Home = ({T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f}) m")

        self.gripper.open_gripper()
        time.sleep(1.0)
        return True

    def run(self):
        log = self.get_logger()
        cv2.namedWindow(self.WINDOW_NAME)

        executor = MultiThreadedExecutor()
        executor.add_node(self)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        if not self.initialize_home():
            return
        self.on_ready()
        log.info(f"=== Ready === {self._key_help_str()}")

        while rclpy.ok():
            # ── Freeze 분기 (pick / scan 진행 중) ──
            if self._frozen_frame is not None:
                vis = self._draw_detections(self._frozen_frame)
                cv2.putText(vis, "[BUSY... CAMERA FROZEN]",
                            (10, 102), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 0, 255), 2)
                cv2.imshow(self.WINDOW_NAME, vis)
                key = cv2.waitKey(30) & 0xFF
                if key == 27:
                    break
                continue

            # ── Live 분기 ──
            if self.color_image is None:
                time.sleep(0.01)
                continue

            frame = self.color_image.copy()
            self._detections = self.run_yolo(frame)

            now = time.time()
            if (self._auto_mode
                    and not self.picking
                    and self.is_auto_ready()
                    and (now - self._last_pick_time) >= cfg.AUTO_PICK_INTERVAL):
                if self._select_target(self._detections) is not None:
                    self._last_pick_time = now
                    self._pick_in_thread(frame)
                    continue

            vis = self._draw_detections(frame)
            cv2.imshow(self.WINDOW_NAME, vis)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            elif key == ord("p"):
                log.info("[KEY] manual pick")
                self._pick_in_thread(frame)
            elif key == ord("a"):
                self._auto_mode = not self._auto_mode
                log.info(f"[KEY] auto {'ON' if self._auto_mode else 'OFF'}")
            else:
                self._handle_key_extra(key)

        cv2.destroyAllWindows()


def run_node(node_cls):
    """공통 main() 헬퍼."""
    rclpy.init()
    node = node_cls()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
