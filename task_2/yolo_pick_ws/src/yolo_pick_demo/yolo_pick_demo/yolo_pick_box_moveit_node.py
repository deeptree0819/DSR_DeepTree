#!/usr/bin/env python3
"""
yolo_pick_box_moveit_node.py
  YOLO Pick & Place — block/gear 를 box 안에 적재.

흐름:
  [Init]    Home 이동 → home_xyz/home_ori 저장
  [Phase 1] (백그라운드) Box 탐색
              - z 상승 → preview → box 검출
              - Home 복귀 → box XY 위로 이동 → 정밀 X, Y 측정
              - self.box_xyz 저장
  [Phase 2] Pick & Place 루프
              - block/gear 검출 → approach + 재검출 → pick
              - box XY 위 SAFE_Z 에서 release
              - Home 복귀

키:
  p   : 1회 픽
  a   : 자동 토글
  s   : box 재탐색
  ESC : 종료
"""

import threading
import time

import cv2
import numpy as np

from . import _config as cfg
from ._base_node import BaseMoveItPickNode, run_node
from ._motion import get_ee_matrix


# ── Box scan 파라미터 ──────────────────────────────
SCAN_Z_OFFSET = 0.15   # home z 위로 추가 높이
SCAN_SETTLE   = 0.4    # 매 이동 후 안정화 [s]
SCAN_PREVIEW  = 1.5    # scan pose 도착 후 시야 표시 [s]


class YoloPickBoxMoveItNode(BaseMoveItPickNode):
    NODE_NAME        = "yolo_pick_box_moveit_node"
    MOVEIT_NODE_NAME = "yolo_pick_box_moveit_py"
    WINDOW_NAME      = "YOLO Pick & Place into Box (MoveIt)"

    def __init__(self):
        super().__init__()
        self.box_xyz = None     # (x, y, z) m — scan_box 가 설정
        self._phase = "INIT"    # INIT / SCAN / READY / PICK

    # ════════════════════════════════════════════
    #  Selection
    # ════════════════════════════════════════════
    def _select_target(self, detections):
        """block 우선(depth asc) → gear(size asc). box 는 픽 대상 아님."""
        blocks = sorted(
            (d for d in detections if d["cls_id"] == cfg.CLS_BLOCK),
            key=lambda d: d["depth"],
        )
        gears = sorted(
            (d for d in detections if d["cls_id"] == cfg.CLS_GEAR),
            key=lambda d: d["size"],
        )
        if blocks:
            return blocks[0]
        if gears:
            return gears[0]
        return None

    def _select_box(self, detections):
        """가장 신뢰도 높은 box 1개."""
        boxes = [d for d in detections if d["cls_id"] == cfg.CLS_BOX]
        if not boxes:
            return None
        return max(boxes, key=lambda d: d["conf"])

    def is_auto_ready(self) -> bool:
        return self.box_xyz is not None

    # ════════════════════════════════════════════
    #  Phase 1: Box scan
    # ════════════════════════════════════════════
    def on_ready(self):
        """Home 이후 자동으로 백그라운드 scan 시작."""
        log = self.get_logger()
        log.info("=== [Phase 1] Box 탐색 시작 (백그라운드) ===")
        time.sleep(1.0)   # 카메라 첫 프레임 대기
        self._scan_in_thread()

    def _scan_in_thread(self):
        if self.picking:
            return

        def _work():
            self.picking = True
            try:
                self.scan_box()
            finally:
                self.picking = False
                self._frozen_frame = None

        threading.Thread(target=_work, daemon=True).start()

    def scan_box(self) -> bool:
        log = self.get_logger()
        self._phase = "SCAN"
        ori = self.home_ori
        hx, hy, hz = self.home_xyz
        scan_z = hz + SCAN_Z_OFFSET

        # 0) Scan pose 이동
        log.info(f"[SCAN] scan pose -> ({hx:.3f}, {hy:.3f}, {scan_z:.3f})")
        if not self.plan_pose(hx, hy, scan_z, ori):
            log.error("[SCAN] scan pose 이동 실패")
            return False
        time.sleep(SCAN_SETTLE)

        # 1) Preview
        if self.color_image is None:
            log.error("[SCAN] 카메라 프레임 없음")
            self._frozen_frame = None
            return False
        preview = self.color_image.copy()
        self._frozen_frame = preview.copy()
        self._detections   = self.run_yolo(preview)
        log.info(f"[SCAN] preview {SCAN_PREVIEW:.1f}s")
        time.sleep(SCAN_PREVIEW)

        # 2) Preview 에서 box 1차 검출
        box_det = self._select_box(self._detections)
        if box_det is None:
            log.error("[SCAN] box 검출 실패")
            self._frozen_frame = None
            return False
        log.info(
            f"[SCAN 1차] box conf={box_det['conf']:.2f} "
            f"center=({box_det['cx']}, {box_det['cy']})"
        )
        base_init = self.pixel_to_base(box_det["cx"], box_det["cy"])
        if base_init is None:
            log.error("[SCAN] 1차 base 변환 실패")
            self._frozen_frame = None
            return False

        # 3) Home 복귀
        log.info("[SCAN] -> Home 복귀")
        if not self.go_home_pose():
            log.error("[SCAN] home 복귀 실패")
            self._frozen_frame = None
            return False
        time.sleep(SCAN_SETTLE)

        # 4) Home 자세에서 box XY 로 이동 (Z = home z)
        cur_z = get_ee_matrix(self.robot)[2, 3]
        log.info(
            f"[SCAN] box XY -> ({base_init[0]:.3f}, {base_init[1]:.3f}, {cur_z:.3f})"
        )
        if not self.plan_pose(base_init[0], base_init[1], cur_z, ori):
            log.error("[SCAN] box XY 이동 실패")
            self._frozen_frame = None
            return False
        time.sleep(SCAN_SETTLE)

        # 5) 정밀 box 재검출
        if self.color_image is None:
            log.error("[SCAN] 재검출 프레임 없음")
            self._frozen_frame = None
            return False
        new_frame = self.color_image.copy()
        self._frozen_frame = new_frame.copy()
        self._detections   = self.run_yolo(new_frame)
        new_box = self._select_box(self._detections)
        if new_box is None:
            log.error("[SCAN] 정렬 후 box 재검출 실패")
            self._frozen_frame = None
            return False

        log.info(
            f"[SCAN 2차] box conf={new_box['conf']:.2f} "
            f"center=({new_box['cx']}, {new_box['cy']})"
        )
        base = self.pixel_to_base(new_box["cx"], new_box["cy"])
        if base is None:
            log.error("[SCAN] 정밀 base 변환 실패")
            self._frozen_frame = None
            return False

        self.box_xyz = base
        log.info(
            f"[SCAN] box pose: ({base[0]:.3f}, {base[1]:.3f}, {base[2]:.3f}) m"
        )

        # 6) Final home
        log.info("[SCAN] -> Home 최종 복귀")
        self.go_home_pose()

        self._frozen_frame = None
        self._phase = "READY"
        return True

    # ════════════════════════════════════════════
    #  Phase 2: Pick (block/gear) → Box 안 place
    # ════════════════════════════════════════════
    def detect_and_pick(self, frame: np.ndarray):
        log = self.get_logger()
        if self.picking:
            log.warn("이미 픽 실행 중. 스킵")
            return
        if self.box_xyz is None:
            log.error("box 위치 없음. 's' 키로 재탐색")
            return

        detections = self.run_yolo(frame)
        self._detections = detections
        target = self._select_target(detections)
        if target is None:
            log.warn("픽 대상(block/gear) 없음")
            return

        log.info(
            f"[YOLO 1차] {target['cls_name']} conf={target['conf']:.2f} "
            f"center=({target['cx']}, {target['cy']}) "
            f"size={target['size']} depth={target['depth']:.3f}"
        )
        base_init = self.pixel_to_base(target["cx"], target["cy"])
        if base_init is None:
            log.error("1차 base 변환 실패")
            return

        self.picking = True
        self._phase = "PICK"
        try:
            new_target = self.approach_and_redetect(
                target["cls_id"], (base_init[0], base_init[1]))
            if new_target is None:
                return
            base = self.pixel_to_base(new_target["cx"], new_target["cy"])
            if base is None:
                log.error("재검출 base 변환 실패")
                return
            self._pick_into_box(*base)
        finally:
            self.picking = False
            self._phase = "READY"

    def _pick_into_box(self, bx, by, bz) -> bool:
        """1)XY → 2)pick_z → 3)close → 4)SAFE_Z → 5)box XY @ SAFE_Z → 6)open → 7)home XY."""
        log = self.get_logger()
        ori = self.home_ori
        pick_z = bz + cfg.Z_OFFSET
        bxp, byp, _ = self.box_xyz

        log.info(f"pick=({bx:.3f},{by:.3f},{bz:.3f}) pick_z={pick_z:.3f} "
                 f"box=({bxp:.3f},{byp:.3f}) drop@SAFE_Z={cfg.SAFE_Z:.3f}")

        cur_z = get_ee_matrix(self.robot)[2, 3]

        self.gripper.open_gripper()
        time.sleep(0.5)

        steps = [
            ("[1] pick XY",  bx,  by,  cur_z),
            ("[2] pick_z",   bx,  by,  pick_z),
        ]
        for label, x, y, z in steps:
            log.info(f"{label} -> ({x:.3f}, {y:.3f}, {z:.3f})")
            if not self.plan_pose(x, y, z, ori):
                log.error(f"{label} 실패"); return False

        log.info("[3] Gripper CLOSE")
        self.gripper.close_gripper()
        time.sleep(1.0)

        steps = [
            ("[4] up SAFE_Z",  bx,  by,  cfg.SAFE_Z),
            ("[5] box XY",     bxp, byp, cfg.SAFE_Z),
        ]
        for label, x, y, z in steps:
            log.info(f"{label} -> ({x:.3f}, {y:.3f}, {z:.3f})")
            if not self.plan_pose(x, y, z, ori):
                log.error(f"{label} 실패"); return False

        log.info("[6] Gripper OPEN")
        self.gripper.open_gripper()
        time.sleep(1.0)

        # home XY 복귀
        hx, hy, _ = self.home_xyz
        log.info(f"[7] home XY -> ({hx:.3f}, {hy:.3f})")
        self.plan_pose(hx, hy, cfg.SAFE_Z, ori)

        log.info("========== PICK END ==========")
        return True

    # ════════════════════════════════════════════
    #  시각화 + 키
    # ════════════════════════════════════════════
    def _draw_detections(self, frame: np.ndarray) -> np.ndarray:
        vis = frame.copy()
        next_target = (self._select_target(self._detections)
                       if self._detections else None)
        for det in self._detections:
            x1, y1, x2, y2 = det["box"]
            if det["cls_id"] == cfg.CLS_BOX:
                color = (255, 200, 0)
                label = f"BOX {det['conf']:.2f}"
            elif det is next_target:
                color = (0, 255, 0)
                label = f"{det['cls_name']} {det['conf']:.2f} <-NEXT"
            else:
                color = (255, 100, 0)
                label = f"{det['cls_name']} {det['conf']:.2f}"
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cv2.putText(vis, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            cv2.drawMarker(vis, (det["cx"], det["cy"]), color,
                           cv2.MARKER_CROSS, 20, 2)

        mode_txt = "AUTO" if self._auto_mode else "MANUAL"
        mode_col = (0, 255, 255) if self._auto_mode else (200, 200, 200)
        cv2.putText(vis,
                    f"[{mode_txt}|{self._phase}] {self._key_help_str()}",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_col, 2)

        if self.box_xyz is not None:
            bx, by, bz = self.box_xyz
            cv2.putText(vis, f"box: ({bx:.3f}, {by:.3f}, {bz:.3f}) m",
                        (10, 52), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 1)
        else:
            cv2.putText(vis, "box: (not scanned)",
                        (10, 52), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 100, 255), 1)
        cv2.putText(vis, f"detections: {len(self._detections)}",
                    (10, 76), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (180, 180, 180), 1)
        return vis

    def _key_help_str(self) -> str:
        return "p:pick a:auto s:scan ESC:quit"

    def _handle_key_extra(self, key: int):
        if key == ord("s"):
            self.get_logger().info("[KEY] box 재탐색")
            self._scan_in_thread()


def main(args=None):
    run_node(YoloPickBoxMoveItNode)


if __name__ == "__main__":
    main()
