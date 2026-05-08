"""MoveIt 모션 유틸 (순수 함수)."""

import numpy as np
from geometry_msgs.msg import PoseStamped

from . import _config as cfg


def clamp_to_safe_workspace(x, y, z, logger):
    """SAFE_* 상수 범위로 클램핑하고 경고 로그."""
    if x < cfg.SAFE_X_MIN:
        logger.warning(f"x={x:.3f} -> {cfg.SAFE_X_MIN}")
        x = cfg.SAFE_X_MIN
    if y < cfg.SAFE_Y_MIN:
        logger.warning(f"y={y:.3f} -> {cfg.SAFE_Y_MIN}")
        y = cfg.SAFE_Y_MIN
    elif y > cfg.SAFE_Y_MAX:
        logger.warning(f"y={y:.3f} -> {cfg.SAFE_Y_MAX}")
        y = cfg.SAFE_Y_MAX
    if z < cfg.SAFE_Z_MIN:
        logger.warning(f"z={z:.3f} -> {cfg.SAFE_Z_MIN}")
        z = cfg.SAFE_Z_MIN
    return x, y, z


def make_pose(x, y, z, ori) -> PoseStamped:
    """(x, y, z) + orientation dict → PoseStamped(base_link)."""
    p = PoseStamped()
    p.header.frame_id = cfg.BASE_FRAME
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = float(z)
    p.pose.orientation.x = ori["x"]
    p.pose.orientation.y = ori["y"]
    p.pose.orientation.z = ori["z"]
    p.pose.orientation.w = ori["w"]
    return p


def get_ee_matrix(moveit_robot) -> np.ndarray:
    """현재 base_link → EE_LINK 4x4 변환행렬."""
    psm = moveit_robot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        T = scene.current_state.get_global_link_transform(cfg.EE_LINK)
    return np.asarray(T, dtype=float)


def plan_and_execute(robot, arm, logger,
                     pose_goal=None, state_goal=None, params=None) -> bool:
    """Pose 또는 RobotState 목표로 plan + execute. 실패 시 False."""
    arm.set_start_state_to_current_state()

    if pose_goal is not None:
        x = pose_goal.pose.position.x
        y = pose_goal.pose.position.y
        z = pose_goal.pose.position.z
        sx, sy, sz = clamp_to_safe_workspace(x, y, z, logger)
        pose_goal.pose.position.x = sx
        pose_goal.pose.position.y = sy
        pose_goal.pose.position.z = sz
        arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=cfg.EE_LINK)
    elif state_goal is not None:
        arm.set_goal_state(robot_state=state_goal)
    else:
        logger.error("plan_and_execute: pose/state 없음")
        return False

    plan_result = arm.plan(parameters=params) if params is not None else arm.plan()
    if not plan_result:
        logger.error("Planning 실패")
        return False

    robot.execute(group_name=cfg.GROUP_NAME,
                  robot_trajectory=plan_result.trajectory,
                  blocking=True)
    return True
