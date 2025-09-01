# sim_app/check_nearest_robot.py
"""
Planning + execution utilities:
- Build a planning grid (without footprint clearing).
- Plan A* path for a given robot.
- Choose the nearest idle robot to a goal.
- Execute a planned path (excute) with replanning and abort handling.

Notes:
- Keeps original constants, prints, and function names (including 'excute').
"""

import asyncio
import math
import numpy as np
from time import time  # kept even if unused

import sim_app.robot_motion as OmniRobotMotion
from sim_app import shared
from sim_app.astar import AStar
from sim_app.astar_env import AStarEnvironment, grid_to_meters, meters_to_grid
from sim_app.map_builder import get_planning_costmap
from sim_app.path_executor import PathExecutor
from sim_app.path_viz import plot_paths_once
from sim_app.robot_controller import OmniRobotController


ROBOT_IDS = ["Rob0", "Rob1", "Rob2"]
ID_TO_MODEL = {"Rob0": "Omnirob0", "Rob1": "Omnirob1", "Rob2": "Omnirob2"}

# Tiny launch/arrival carve (cells)
CLEAR_START_GOAL_RADIUS = 1


# -----------------------------------------------------------------------------
# Planning grid
# -----------------------------------------------------------------------------

def _current_planning_grid() -> np.ndarray:
    """
    Build a planning view (no footprint clearing).
    """
    if shared.global_occupancy is None or shared.global_occupancy.size == 0:
        return np.zeros((shared.GRID_SIZE, shared.GRID_SIZE), dtype=np.float32)

    grid = get_planning_costmap(
        inflation_radius_m=shared.INFLATION_RADIUS_M
    ).astype(np.float32)
    return grid


# -----------------------------------------------------------------------------
# Planner
# -----------------------------------------------------------------------------

async def plan_path(start_pos, goal_pos, robot_name):
    """
    Plan an A* path in meters between start_pos and goal_pos for robot_name.
    Returns (path_in_meters, planning_grid) or (None, None) if planning fails.
    """
    if not shared.FREEZE_MAP:
        shared.ensure_map_covers(
            [start_pos[0], goal_pos[0]],
            [start_pos[1], goal_pos[1]],
            margin_m=1.0,
        )

    grid = _current_planning_grid()
    if grid.size == 0:
        print("⛔ Planning grid is empty. Did you load the saved map?")
        return None, None

    res = shared.MAP_RESOLUTION
    start_grid = meters_to_grid(start_pos[0], start_pos[1], grid, res)
    goal_grid = meters_to_grid(goal_pos[0], goal_pos[1], grid, res)

    H, W = grid.shape
    for (gx, gy), tag in ((start_grid, "start"), (goal_grid, "goal")):
        if not (0 <= gx < W and 0 <= gy < H):
            print(f"⛔ {tag} grid {gx,gy} lies outside the loaded map {W}x{H}.")
            return None, None

    env = AStarEnvironment(
        grid,
        start_grid,
        goal_grid,
        res,
        block_threshold=0.99,        # keep binary blocking strict
        soft_cost_gain=None,         # using proximity cost instead
        proximity_k_cells=3,         # rings to look around each neighbor (3 * 0.20 = 0.60 m)
        proximity_cost_gain=0.5,     # tune 0.5–2.0; higher = keeps farther from walls
    )

    path_g = await asyncio.to_thread(AStar(env).search, "robot")
    if not path_g:
        print("❌ No path found during planning!")
        return None, None

    path_m = [grid_to_meters(x, y, grid, res) for (x, y) in path_g]
    print(f"🚦 A* planned path with {len(path_m)} waypoints.")
    shared.latest_astar_path_by_robot[robot_name] = list(path_m)
    return path_m, grid


# -----------------------------------------------------------------------------
# Robot selection
# -----------------------------------------------------------------------------

async def find_available_robot(sim, goal_pos):
    """
    Loop over Rob0/Rob1/Rob2, pick the nearest IDLE robot.
    Returns (robot_id, start_pos) or (None, None).
    """
    candidates = []

    for rid in ROBOT_IDS:
        # treat missing status as idle; adjust if you use a different convention
        if shared.robot_status.get(rid) not in (None, "idle"):
            continue

        model = ID_TO_MODEL[rid]
        ctrl = OmniRobotController(sim)
        try:
            await ctrl.init_handles(sim, robot_name=model)

            # Support both controller signatures:
            #   get_position()              (no-arg)
            #   get_position(robot_handle)  (legacy)
            try:
                pos = await ctrl.get_position()
            except TypeError:
                pos = await ctrl.get_position(ctrl.robot)

        except Exception as e:
            print(f"⚠️ Skipping {rid} ({model}): {e}")
            continue

        dist = math.hypot(goal_pos[0] - pos[0], goal_pos[1] - pos[1])
        candidates.append((rid, pos, dist))

    if not candidates:
        print("⛔ No available idle robots.")
        return None, None

    robot_id, start_pos, _ = min(candidates, key=lambda x: x[2])
    print(f"✅ Nearest available robot: {robot_id} at {start_pos}")
    return robot_id, start_pos


# -----------------------------------------------------------------------------
# Execution
# -----------------------------------------------------------------------------

async def excute(sim, start_pos, start_ori, robot_name, goal_pos):
    """
    Execute motion for `robot_name` from start_pos/orientation to goal_pos.
    Handles replanning, aborts, and final state updates. Returns "DONE" or "FAILED".
    """
    model = ID_TO_MODEL.get(robot_name)
    if not model:
        print(f"⚠️ Unknown robot name: {robot_name}")
        return "FAILED"

    controller = OmniRobotController(sim)
    await controller.init_handles(sim, robot_name=model)

    shared.robot_name = robot_name
    shared.robot_goal[robot_name] = tuple(goal_pos)
    shared.robot_status[robot_name] = "busy"

    motion = OmniRobotMotion.RobotMotion(sim, controller.wheels)  # create once
    t0 = None
    max_replans = 8
    for _ in range(max_replans):
        # hard stop requested? stop wheels and exit
        if shared.robot_abort.get(robot_name):
            shared.robot_abort[robot_name] = False
            await motion.stop()
            shared.robot_status[robot_name] = "idle"
            if t0 is not None:
                elapsed = time() - t0
                msg = f"⏱️ {robot_name} mission aborted after {elapsed:.2f}s"
                print(msg)
                shared.message_log.append(msg)
            return "FAILED"
        if t0 is None:
            t0 = time()

        # A* offloaded so other robots can plan too
        path_in_meters, plan_grid = await plan_path(start_pos, goal_pos, robot_name)

        if not path_in_meters:
            await motion.stop()
            shared.robot_status[robot_name] = "idle"
            if t0 is not None:
                elapsed = time() - t0
                msg = f"⏱️ {robot_name} mission failed after {elapsed:.2f}s"
                print(msg)
                shared.message_log.append(msg)
            return "FAILED"

        executor = PathExecutor(sim, robot_name, controller.wheels, controller.robot, plan_grid)

        # follow the path; executor itself will also honor abort
        result = await executor.follow_path(path_in_meters)

        if shared.robot_abort.get(robot_name):
            shared.robot_abort[robot_name] = False
            await motion.stop()
            shared.robot_status[robot_name] = "idle"
            plot_paths_once(robot_name)
            if t0 is not None:
                elapsed = time() - t0
                msg = f"⏱️ {robot_name} mission aborted after {elapsed:.2f}s"
                print(msg)
                shared.message_log.append(msg)
            return "FAILED"

        if result == "replanned":
            start_pos = await controller.get_position()
            continue

        if result == "DONE":
            await motion.stop()
            shared.robot_status[robot_name] = "idle"
            plot_paths_once(robot_name)
            if t0 is not None:
                elapsed = time() - t0
                msg = f"⏱️ {robot_name} mission completed in {elapsed:.2f}s"
                print(msg)
                shared.message_log.append(msg)
            return "DONE"

        break

    await motion.stop()
    shared.robot_status[robot_name] = "idle"
    plot_paths_once(robot_name)
    if t0 is not None:
        elapsed = time() - t0
        msg = f"⏱️ {robot_name} mission ended after {elapsed:.2f}s (fallthrough)"
        print(msg)
        shared.message_log.append(msg)
    return "FAILED"
