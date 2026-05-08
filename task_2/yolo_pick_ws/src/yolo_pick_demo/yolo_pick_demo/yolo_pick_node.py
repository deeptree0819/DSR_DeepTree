#!/usr/bin/env python3
"""
yolo_pick_node.py
기존 click_pick_node.py 의 마우스 클릭 좌표 입력을
YOLO 객체 검출 좌표로 대체한 노드.

변경된 부분:
  - mouse_callback() 제거
  - YOLO 모델 로드 및 detect_and_pick() 추가
  - 키 입력:
      p  : 현재 프레임에서 신뢰도 최고 객체를 1개 집기
      a  : 자동 모드 토글 (일정 간격으로 자동 집기)
      ESC: 종료

유지된 부분 (click_pick_node.py 와 동일):
  - transform_to_base()
  - pick_and_place()
  - clamp_to_safe_workspace()
  - get_robot_pose_matrix()
  - ROS 구독 / 카메라 콜백 전체
  - 그리퍼 / Doosan API 초기화
"""

import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from .onrobot import RG
import DR_init

try:
    from ultralytics import YOLO
except ImportError:
    raise ImportError("pip install ultralytics")


# ─────────────────────────────────────────────
#  로봇 / 그리퍼 설정 (click_pick_node 와 동일)
# ─────────────────────────────────────────────
ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

GRIPPER_NAME     = "rg2"
TOOLCHARGER_IP   = "192.168.1.1"
TOOLCHARGER_PORT = 502

# ─────────────────────────────────────────────
#  Z 관련 파라미터 (test.py 와 동일)
# ─────────────────────────────────────────────
Z_OFFSET       = 220.0   # 검출 지점 z에 더해 줄 오프셋 [mm]
SAFE_Z         = 400.0   # 집은 뒤 올라갈 안전 높이 [mm]
PLACE_Z_FLOOR  = 250.0   # 홈 위치에서 내려갈 최소 높이 [mm] (z_final = max(이 값, z_correct))

# ─────────────────────────────────────────────
#  YOLO 설정
# ─────────────────────────────────────────────
YOLO_MODEL_PATH  = "/home/ssu/yolo_pick_ws/best.pt"  # bar / gear 커스텀 모델
YOLO_CONF_THRESH = 0.5           # 최소 신뢰도
YOLO_TARGET_CLS  = None          # None = 전체 클래스, 정수 = 특정 클래스 ID만 (0=bar, 1=gear)
AUTO_PICK_INTERVAL = 3.0         # 자동 모드 집기 간격 [s]


class YoloPickNode(Node):

    def __init__(self):
        super().__init__("yolo_pick_node", namespace=ROBOT_ID)
        setattr(DR_init, "__dsr__node", self)

        # ── Doosan 함수 import ───────────────────────────
        try:
            from DSR_ROBOT2 import get_current_posx, movej, movel, wait
            from DR_common2 import posx, posj
        except ImportError as e:
            self.get_logger().error(f"DSR_ROBOT2 import 실패: {e}")
            raise

        self.get_current_posx = get_current_posx
        self.movej = movej
        self.movel = movel
        self.wait  = wait
        self.posx  = posx
        self.posj  = posj

        # ── OpenCV bridge ────────────────────────────────
        self.bridge = CvBridge()

        # ── 최신 프레임 / Intrinsic ──────────────────────
        self.color_image = None
        self.depth_image = None
        self.intrinsics  = None   # {fx, fy, ppx, ppy}

        # ── Hand-Eye 변환행렬 로드 ───────────────────────
        calib_file = (
            Path(get_package_share_directory("yolo_pick_demo"))
            / "config"
            / "T_gripper2camera.npy"
        )
        self.gripper2cam = np.load(str(calib_file))
        self.get_logger().info(f"Hand-Eye 행렬 로드 완료: {calib_file}")

        # ── 그리퍼 ──────────────────────────────────────
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

        # ── 초기 자세 ────────────────────────────────────
        self.home_pose = None
        self.JReady    = self.posj([0, 0, 90, 0, 90, 90])

        # ── YOLO 모델 로드 ───────────────────────────────
        self.get_logger().info(f"YOLO 모델 로드 중: {YOLO_MODEL_PATH}")
        self.yolo = YOLO(YOLO_MODEL_PATH)
        self.get_logger().info("YOLO 모델 로드 완료")

        # ── 자동 모드 상태 ───────────────────────────────
        self._auto_mode      = False
        self._last_pick_time = 0.0
        self._is_picking     = False   # 픽 실행 중 플래그 (중복 방지)

        # 최신 YOLO 검출 결과 (시각화용)
        self._detections: list[dict] = []  # {cx, cy, conf, cls_name, box}

        # ── ROS 구독 ─────────────────────────────────────
        self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info",
            self.camera_info_callback, 10,
        )
        self.create_subscription(
            Image, "/camera/camera/color/image_raw",
            self.color_image_callback, 10,
        )
        self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_image_callback, 10,
        )

    # ════════════════════════════════════════════════
    #  카메라 콜백 (click_pick_node 와 동일)
    # ════════════════════════════════════════════════
    def camera_info_callback(self, msg: CameraInfo):
        self.intrinsics = {
            "fx": msg.k[0], "fy": msg.k[4],
            "ppx": msg.k[2], "ppy": msg.k[5],
        }

    def color_image_callback(self, msg: Image):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def depth_image_callback(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # ════════════════════════════════════════════════
    #  좌표 변환 (test.py 와 동일)
    # ════════════════════════════════════════════════
    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3]  = [x, y, z]
        return T

    def transform_to_base(self, camera_coords):
        coord = np.append(np.array(camera_coords, dtype=float), 1.0)
        base2gripper = self.get_robot_pose_matrix(*self.get_current_posx()[0])
        base2cam     = base2gripper @ self.gripper2cam
        td_coord     = base2cam @ coord
        return td_coord[:3]

    def pick_and_place(self, x, y, z):
        """test.py 의 pick_and_place() 시퀀스와 동일."""
        log = self.get_logger()
        log.info("========== PICK SEQUENCE ==========")
        log.info(f"Base coord raw : x={x:.2f}, y={y:.2f}, z={z:.2f}")
        log.info(f"Z_OFFSET       : {Z_OFFSET}")
        log.info(f"SAFE_Z         : {SAFE_Z}")
        log.info("===================================")

        cur = self.get_current_posx()[0]
        cur_x, cur_y, cur_z, rx, ry, rz = cur

        if self.home_pose is None:
            self.home_pose = cur
        home_x, home_y, _, hrx, hry, hrz = self.home_pose

        # 0) 안전용 open
        self.gripper.open_gripper()
        time.sleep(0.5)

        # 1) XY 만 이동 (z 는 현재값 유지)
        target_xy = self.posx([x, y, cur_z, rx, ry, rz])
        log.info(f"[1] Move to XY only: {target_xy}")
        self.movel(target_xy, VELOCITY, ACC);  self.wait(0.5)

        # 2) z_correct = z + Z_OFFSET 으로 하강
        z_correct = z + Z_OFFSET
        target_xyz = self.posx([x, y, z_correct, rx, ry, rz])
        log.info(f"[2] Move down to z_correct={z_correct:.2f}: {target_xyz}")
        self.movel(target_xyz, VELOCITY, ACC);  self.wait(0.3)

        # 3) Gripper Close
        log.info("[3] Gripper Close")
        self.gripper.close_gripper();  time.sleep(1.0)

        # 4) SAFE_Z 까지 상승
        up_pose = self.posx([x, y, SAFE_Z, rx, ry, rz])
        log.info(f"[4] Move up to SAFE_Z={SAFE_Z:.2f}: {up_pose}")
        self.movel(up_pose, VELOCITY, ACC);  self.wait(0.5)

        # 5) home XY 로 이동 (z 는 SAFE_Z 유지)
        home_xy_pose = self.posx([home_x, home_y, SAFE_Z, hrx, hry, hrz])
        log.info(f"[5] Move to home XY: {home_xy_pose}")
        self.movel(home_xy_pose, VELOCITY, ACC);  self.wait(0.5)

        # 6) place 높이 = pick 높이 이상으로 보장
        z_final = max(PLACE_Z_FLOOR, z_correct)
        home_place = self.posx([home_x, home_y, z_final, hrx, hry, hrz])
        log.info(f"[6] Move to home XY, z={z_final}: {home_place}")
        self.movel(home_place, VELOCITY, ACC);  self.wait(0.3)

        # 7) Gripper Open
        log.info("[7] Gripper Open")
        self.gripper.open_gripper();  time.sleep(1.0)

        # 8) 다시 SAFE_Z 로 복귀
        back_up = self.posx([home_x, home_y, SAFE_Z, hrx, hry, hrz])
        log.info(f"[8] Back up to SAFE_Z: {back_up}")
        self.movel(back_up, VELOCITY, ACC);  self.wait(0.5)

        log.info("========== PICK END ==========")

    # ════════════════════════════════════════════════
    #  ★ YOLO 검출 (마우스 콜백 대체 핵심 로직) ★
    # ════════════════════════════════════════════════
    def _pixel_to_base(self, px: int, py: int) -> tuple[float, float, float] | None:
        """
        픽셀 (px, py) → Depth 조회 → 카메라 3D 좌표 → 베이스 좌표.
        click_pick_node 의 mouse_callback 내부 로직과 동일.
        """
        if self.depth_image is None or self.intrinsics is None:
            return None

        h, w = self.depth_image.shape
        if not (0 <= px < w and 0 <= py < h):
            self.get_logger().warn("픽셀 범위 초과")
            return None

        z_raw = self.depth_image[py, px]
        if z_raw == 0:
            self.get_logger().warn(f"Depth 값 없음 at ({px}, {py})")
            return None

        # 16UC1(mm) vs 32FC1(m) 처리
        z_mm = float(z_raw) if self.depth_image.dtype == np.uint16 else float(z_raw) * 1000.0

        fx, fy   = self.intrinsics["fx"],  self.intrinsics["fy"]
        ppx, ppy = self.intrinsics["ppx"], self.intrinsics["ppy"]

        X = (px - ppx) * z_mm / fx
        Y = (py - ppy) * z_mm / fy
        Z = z_mm

        cam_coord  = (X, Y, Z)
        base_coord = self.transform_to_base(cam_coord)

        self.get_logger().info(
            f"Camera: ({X:.1f}, {Y:.1f}, {Z:.1f}) mm  →  "
            f"Base: ({base_coord[0]:.1f}, {base_coord[1]:.1f}, {base_coord[2]:.1f}) mm"
        )
        return tuple(float(v) for v in base_coord)

    def _run_yolo(self, frame: np.ndarray) -> list[dict]:
        """
        YOLO 추론 실행.
        반환: [{cx, cy, conf, cls_id, cls_name, box(x1,y1,x2,y2)}, ...]
        """
        results = self.yolo(frame, verbose=False)[0]
        detections = []

        for box in results.boxes:
            conf   = float(box.conf[0])
            cls_id = int(box.cls[0])

            if conf < YOLO_CONF_THRESH:
                continue
            if YOLO_TARGET_CLS is not None and cls_id != YOLO_TARGET_CLS:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cls_name = self.yolo.names.get(cls_id, str(cls_id))

            detections.append({
                "cx": cx, "cy": cy,
                "conf": conf,
                "cls_id": cls_id,
                "cls_name": cls_name,
                "box": (x1, y1, x2, y2),
            })

        # 신뢰도 내림차순 정렬
        detections.sort(key=lambda d: d["conf"], reverse=True)
        return detections

    def detect_and_pick(self, frame: np.ndarray):
        """
        YOLO 검출 후 신뢰도 1위 객체를 집는다.
        click_pick_node 의 mouse_callback → pick_and_place 흐름과 동일.
        """
        if self._is_picking:
            self.get_logger().warn("이미 픽 실행 중. 스킵.")
            return

        detections = self._run_yolo(frame)
        self._detections = detections   # 시각화용 갱신

        if not detections:
            self.get_logger().warn("검출된 객체 없음.")
            return

        best = detections[0]
        self.get_logger().info(
            f"[YOLO] 최고 신뢰도 검출: {best['cls_name']} "
            f"conf={best['conf']:.2f}  center=({best['cx']}, {best['cy']})"
        )

        base_coord = self._pixel_to_base(best["cx"], best["cy"])
        if base_coord is None:
            self.get_logger().error("베이스 좌표 변환 실패. 픽 취소.")
            return

        bx, by, bz = base_coord

        self._is_picking = True
        try:
            self.pick_and_place(bx, by, bz)
        finally:
            self._is_picking = False

    # ════════════════════════════════════════════════
    #  시각화 헬퍼
    # ════════════════════════════════════════════════
    def _draw_detections(self, frame: np.ndarray) -> np.ndarray:
        """YOLO 검출 박스 + 레이블 + 크로스헤어를 프레임에 그린다."""
        vis = frame.copy()

        for i, det in enumerate(self._detections):
            x1, y1, x2, y2 = det["box"]
            cx, cy = det["cx"], det["cy"]
            label  = f"{det['cls_name']} {det['conf']:.2f}"

            # 1위 = 초록, 나머지 = 파랑
            color = (0, 255, 0) if i == 0 else (255, 100, 0)

            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cv2.putText(vis, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            # 중심점 크로스헤어
            cv2.drawMarker(vis, (cx, cy), color,
                           cv2.MARKER_CROSS, 20, 2)

        # 모드 표시
        mode_txt = "AUTO" if self._auto_mode else "MANUAL"
        mode_col = (0, 255, 255) if self._auto_mode else (200, 200, 200)
        cv2.putText(vis, f"[{mode_txt}]  p:pick  a:auto  ESC:quit",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_col, 2)
        cv2.putText(vis,
                    f"detections: {len(self._detections)}",
                    (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        return vis

    # ════════════════════════════════════════════════
    #  메인 루프
    # ════════════════════════════════════════════════
    def run(self):
        window = "YOLO Pick & Place"
        cv2.namedWindow(window)

        # 초기 자세 이동
        self.get_logger().info("[Init] movej JReady")
        self.movej(self.JReady, VELOCITY, ACC)
        self.wait(1.0)
        self.home_pose = self.get_current_posx()[0]

        self.get_logger().info("[Init] Gripper Open")
        self.gripper.open_gripper()
        time.sleep(1.0)

        self.get_logger().info("=== 준비 완료 ===  p: 수동 픽  a: 자동 토글  ESC: 종료")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.color_image is None:
                continue

            frame = self.color_image.copy()

            # YOLO 추론 (항상 실행해서 박스 시각화)
            self._detections = self._run_yolo(frame)

            # 자동 모드: 일정 간격으로 자동 픽
            now = time.time()
            if (self._auto_mode
                    and not self._is_picking
                    and (now - self._last_pick_time) >= AUTO_PICK_INTERVAL):
                self._last_pick_time = now
                self.detect_and_pick(frame)

            # 시각화
            vis = self._draw_detections(frame)
            cv2.imshow(window, vis)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:                # ESC
                break
            elif key == ord('p'):        # 수동 픽
                self.get_logger().info("[KEY] 수동 픽 트리거")
                self.detect_and_pick(frame)
            elif key == ord('a'):        # 자동 모드 토글
                self._auto_mode = not self._auto_mode
                self.get_logger().info(
                    f"[KEY] 자동 모드 {'ON' if self._auto_mode else 'OFF'}"
                )

        cv2.destroyAllWindows()


# ════════════════════════════════════════════════
#  엔트리포인트
# ════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = YoloPickNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
