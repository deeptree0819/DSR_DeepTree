#!/usr/bin/env python3
"""
yolo_pick_moveit_node.py
  YOLO 검출 → 신뢰도 1위 객체 1개 → home XY 위치에 place.
  베이스: BaseMoveItPickNode

키:
  p   : 1회 픽
  a   : 자동 토글
  ESC : 종료
"""

import time

import numpy as np

from . import _config as cfg
from ._base_node import BaseMoveItPickNode, run_node


# 노드 고유: place 시 최소 높이 (pick_z 이상 보장)
PLACE_Z_FLOOR = 0.25


class YoloPickMoveItNode(BaseMoveItPickNode):
    NODE_NAME        = "yolo_pick_moveit_node"
    MOVEIT_NODE_NAME = "yolo_pick_moveit_py"
    WINDOW_NAME      = "YOLO Pick & Place (MoveIt)"

    def _select_target(self, detections):
        """가장 신뢰도 높은 객체."""
        if not detections:
            return None
        return max(detections, key=lambda d: d["conf"])

    # ── Pick → home XY place ──
    def detect_and_pick(self, frame: np.ndarray):
        log = self.get_logger()
        if self.picking:
            log.warn("이미 픽 실행 중. 스킵")
            return

        detections = self.run_yolo(frame)
        self._detections = detections
        target = self._select_target(detections)
        if target is None:
            log.warn("검출 없음")
            return

        log.info(
            f"[YOLO] target={target['cls_name']} conf={target['conf']:.2f} "
            f"center=({target['cx']}, {target['cy']})"
        )

        base = self.pixel_to_base(target["cx"], target["cy"])
        if base is None:
            log.error("base 변환 실패. 픽 취소.")
            return
        bx, by, bz = base

        self.picking = True
        try:
            self._pick_to_home(bx, by, bz)
        finally:
            self.picking = False

    def _pick_to_home(self, bx, by, bz):
        """1)XY → 2)pick_z → 3)close → 4)SAFE_Z → 5)home XY → 6)place_z → 7)open → 8)SAFE_Z."""
        log = self.get_logger()
        ori = self.home_ori
        pick_z  = bz + cfg.Z_OFFSET
        place_z = max(pick_z, PLACE_Z_FLOOR)
        hx, hy, _ = self.home_xyz

        log.info(f"pick_z={pick_z:.3f}, place_z={place_z:.3f}")

        from ._motion import get_ee_matrix
        cur_z = get_ee_matrix(self.robot)[2, 3]

        self.gripper.open_gripper()
        time.sleep(0.5)

        steps = [
            ("[1] XY",       bx, by, cur_z),
            ("[2] pick_z",   bx, by, pick_z),
        ]
        for label, x, y, z in steps:
            log.info(f"{label} -> ({x:.3f}, {y:.3f}, {z:.3f})")
            if not self.plan_pose(x, y, z, ori):
                log.error(f"{label} 실패"); return

        log.info("[3] Gripper CLOSE")
        self.gripper.close_gripper()
        time.sleep(1.0)

        steps = [
            ("[4] up SAFE_Z",  bx, by, cfg.SAFE_Z),
            ("[5] home XY",    hx, hy, cfg.SAFE_Z),
            ("[6] place_z",    hx, hy, place_z),
        ]
        for label, x, y, z in steps:
            log.info(f"{label} -> ({x:.3f}, {y:.3f}, {z:.3f})")
            if not self.plan_pose(x, y, z, ori):
                log.error(f"{label} 실패"); return

        log.info("[7] Gripper OPEN")
        self.gripper.open_gripper()
        time.sleep(1.0)

        log.info(f"[8] up SAFE_Z")
        self.plan_pose(hx, hy, cfg.SAFE_Z, ori)
        log.info("========== PICK END ==========")


def main(args=None):
    run_node(YoloPickMoveItNode)


if __name__ == "__main__":
    main()
