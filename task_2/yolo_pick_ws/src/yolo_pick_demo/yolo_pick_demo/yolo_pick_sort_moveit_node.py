#!/usr/bin/env python3
"""
yolo_pick_sort_moveit_node.py
  YOLO 검출 → block / gear 를 1열로 정렬해 place.

  block: depth 오름차순 (가까운 것 → block[0])
  gear : size  오름차순 (작은 것 → gear[0])
  1열 (PLACE_Y 고정), block 3 + gear 2 = 5 슬롯.
  place_z = pick_z (집은 그 높이 그대로).

키:
  p   : 1회 픽
  a   : 자동 토글
  r   : 슬롯 카운터 리셋
  ESC : 종료
"""

import time

import cv2
import numpy as np

from . import _config as cfg
from ._base_node import BaseMoveItPickNode, run_node


# ── Place 슬롯: 1열 (m, base_link) ──────────────────
PLACE_Y         = 0.27
BLOCK_X0        = 0.30
BLOCK_DX        = 0.05
BLOCK_NUM_SLOTS = 3

BLOCK_GEAR_GAP  = 0.10
GEAR_X0         = (BLOCK_X0 + (BLOCK_NUM_SLOTS - 1) * BLOCK_DX
                   + BLOCK_GEAR_GAP)
GEAR_DX         = 0.08
GEAR_NUM_SLOTS  = 2


class YoloPickSortMoveItNode(BaseMoveItPickNode):
    NODE_NAME        = "yolo_pick_sort_moveit_node"
    MOVEIT_NODE_NAME = "yolo_pick_sort_moveit_py"
    WINDOW_NAME      = "YOLO Pick & Sort Place (MoveIt)"

    def __init__(self):
        super().__init__()
        self.block_count = 0
        self.gear_count  = 0

    # ── 슬롯 ──
    def _next_place_xy(self, cls_id):
        """다음 슬롯 (x, y), 인덱스 반환. 가득 차면 (None, idx)."""
        if cls_id == cfg.CLS_BLOCK:
            idx = self.block_count
            if idx >= BLOCK_NUM_SLOTS:
                return None, idx
            return (BLOCK_X0 + idx * BLOCK_DX, PLACE_Y), idx
        if cls_id == cfg.CLS_GEAR:
            idx = self.gear_count
            if idx >= GEAR_NUM_SLOTS:
                return None, idx
            return (GEAR_X0 + idx * GEAR_DX, PLACE_Y), idx
        return None, -1

    def _commit_slot(self, cls_id):
        if cls_id == cfg.CLS_BLOCK:
            self.block_count += 1
        elif cls_id == cfg.CLS_GEAR:
            self.gear_count += 1

    # ── Selection ──
    def _select_target(self, detections):
        """block 우선(depth asc) → gear(size asc). 슬롯 가득 차면 다음 클래스."""
        blocks = sorted(
            (d for d in detections if d["cls_id"] == cfg.CLS_BLOCK),
            key=lambda d: d["depth"],
        )
        gears = sorted(
            (d for d in detections if d["cls_id"] == cfg.CLS_GEAR),
            key=lambda d: d["size"],
        )
        if self.block_count < BLOCK_NUM_SLOTS and blocks:
            return blocks[0]
        if self.gear_count < GEAR_NUM_SLOTS and gears:
            return gears[0]
        return None

    # ── Pick + Place ──
    def detect_and_pick(self, frame: np.ndarray):
        log = self.get_logger()
        if self.picking:
            log.warn("이미 픽 실행 중. 스킵")
            return

        detections = self.run_yolo(frame)
        self._detections = detections
        target = self._select_target(detections)
        if target is None:
            log.warn(
                f"슬롯 가득 또는 검출 없음 — block {self.block_count}/{BLOCK_NUM_SLOTS}, "
                f"gear {self.gear_count}/{GEAR_NUM_SLOTS}"
            )
            return

        place_xy, slot_idx = self._next_place_xy(target["cls_id"])
        if place_xy is None:
            log.warn("슬롯 없음")
            return

        log.info(
            f"[YOLO 1차] {target['cls_name']} conf={target['conf']:.2f} "
            f"-> slot[{target['cls_name']}][{slot_idx}] = {place_xy}"
        )

        base_init = self.pixel_to_base(target["cx"], target["cy"])
        if base_init is None:
            log.error("1차 base 변환 실패")
            return

        self.picking = True
        try:
            new_target = self.approach_and_redetect(
                target["cls_id"], (base_init[0], base_init[1]))
            if new_target is None:
                return

            base = self.pixel_to_base(new_target["cx"], new_target["cy"])
            if base is None:
                log.error("재검출 base 변환 실패")
                return
            bx, by, bz = base
            px, py = place_xy

            if self._pick_and_place(bx, by, bz, px, py):
                self._commit_slot(target["cls_id"])
        finally:
            self.picking = False

    def _pick_and_place(self, bx, by, bz, px, py) -> bool:
        log = self.get_logger()
        ori = self.home_ori
        pick_z  = bz + cfg.Z_OFFSET
        place_z = pick_z   # place z = pick z

        log.info(f"pick_z={pick_z:.3f} (=place_z)  pick=({bx:.3f},{by:.3f},{bz:.3f}) "
                 f"place=({px:.3f},{py:.3f})")

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
                log.error(f"{label} 실패"); return False

        log.info("[3] Gripper CLOSE")
        self.gripper.close_gripper()
        time.sleep(1.0)

        steps = [
            ("[4] up SAFE_Z",  bx, by, cfg.SAFE_Z),
            ("[5] place XY",   px, py, cfg.SAFE_Z),
            ("[6] place_z",    px, py, place_z),
        ]
        for label, x, y, z in steps:
            log.info(f"{label} -> ({x:.3f}, {y:.3f}, {z:.3f})")
            if not self.plan_pose(x, y, z, ori):
                log.error(f"{label} 실패"); return False

        log.info("[7] Gripper OPEN")
        self.gripper.open_gripper()
        time.sleep(1.0)

        log.info(f"[8] up SAFE_Z")
        self.plan_pose(px, py, cfg.SAFE_Z, ori)

        # home XY 복귀 (다음 검출 시점 회복)
        hx, hy, _ = self.home_xyz
        log.info(f"[9] home XY -> ({hx:.3f}, {hy:.3f})")
        self.plan_pose(hx, hy, cfg.SAFE_Z, ori)

        log.info("========== PICK END ==========")
        return True

    # ── 시각화 (override: 클래스별 라벨 + 슬롯 상태) ──
    def _draw_detections(self, frame: np.ndarray) -> np.ndarray:
        vis = frame.copy()
        next_target = (self._select_target(self._detections)
                       if self._detections else None)
        for det in self._detections:
            x1, y1, x2, y2 = det["box"]
            if det["cls_id"] == cfg.CLS_BLOCK:
                d_txt = (f"d={det['depth']:.2f}m"
                         if det["depth"] != float("inf") else "d=?")
                label = f"{det['cls_name']} {d_txt} ({det['conf']:.2f})"
            else:
                label = f"{det['cls_name']} sz={det['size']} ({det['conf']:.2f})"
            color = (0, 255, 0) if det is next_target else (255, 100, 0)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cv2.putText(vis, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            cv2.drawMarker(vis, (det["cx"], det["cy"]), color,
                           cv2.MARKER_CROSS, 20, 2)

        mode_txt = "AUTO" if self._auto_mode else "MANUAL"
        mode_col = (0, 255, 255) if self._auto_mode else (200, 200, 200)
        cv2.putText(vis, f"[{mode_txt}] {self._key_help_str()}",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_col, 2)
        cv2.putText(vis,
                    f"slots - block {self.block_count}/{BLOCK_NUM_SLOTS}  "
                    f"gear {self.gear_count}/{GEAR_NUM_SLOTS}",
                    (10, 52), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (200, 200, 200), 1)
        cv2.putText(vis,
                    f"sort: block(depth asc), gear(size asc)  "
                    f"detections: {len(self._detections)}",
                    (10, 76), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (180, 180, 180), 1)
        return vis

    def _key_help_str(self) -> str:
        return "p:pick a:auto r:reset ESC:quit"

    def _handle_key_extra(self, key: int):
        if key == ord("r"):
            self.block_count = 0
            self.gear_count  = 0
            self.get_logger().info("[KEY] 슬롯 카운터 리셋")


def main(args=None):
    run_node(YoloPickSortMoveItNode)


if __name__ == "__main__":
    main()
