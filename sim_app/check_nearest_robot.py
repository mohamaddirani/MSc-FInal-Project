# sim_app/check_nearest_robot.py
import math
import numpy as np
from time import time

from sim_app.robot_controller import OmniRobotController
from sim_app import shared
from sim_app.astar_env import AStarEnvironment, grid_to_meters, meters_to_grid
from sim_app.path_executor import PathExecutor
from sim_app.astar import AStar
import sim_app.robot_motion as OmniRobotMotion


ROBOT_IDS = ["Rob0", "Rob1", "Rob2"]
ID_TO_MODEL = {"Rob0": "Omnirob0", "Rob1": "Omnirob1", "Rob2": "Omnirob2"}


def _current_planning_grid() -> np.ndarray:
    cm = shared.global_costmap
    return cm.astype(np.float32) if cm is not None and cm.size else \
           np.zeros((shared.GRID_SIZE, shared.GRID_SIZE), dtype=np.float32)


async def plan_path(start_pos, goal_pos, robot_name):
    # Grow the map if needed around start/goal
    shared.ensure_map_covers([start_pos[0], goal_pos[0]],
                             [start_pos[1], goal_pos[1]], margin_m=1.0)

    grid = _current_planning_grid()
    res  = shared.MAP_RESOLUTION

    start_grid = meters_to_grid(start_pos[0], start_pos[1], grid, res)
    goal_grid  = meters_to_grid(goal_pos[0],  goal_pos[1],  grid, res)

    print(f"📌 Planning path: Start Grid {start_grid}, Goal Grid {goal_grid}")
    occ = (shared.global_occupancy > 0).astype(np.float32)
    env = AStarEnvironment(
        occ,
        start_grid,
        goal_grid,
        res,
        block_threshold=0.5,      # 1’s are blocked, 0’s free
        soft_cost_gain=None
    )

    path_g = AStar(env).search("robot")
    if not path_g:
        print("❌ No path found during planning!")
        return None

    path_m = [grid_to_meters(x, y, grid, res) for (x, y) in path_g]
    print(f"🚦 A* planned path with {len(path_m)} waypoints.")
    return path_m


async def find_available_robot(sim, goal_pos):
    """
    Loop over Rob0/Rob1/Rob2, pick the nearest IDLE robot.
    Returns (robot_id, start_pos) or (None, None).
    """
    candidates = []

    for rid in ROBOT_IDS:
        if shared.robot_status.get(rid) != "idle":
            continue

        model = ID_TO_MODEL[rid]
        ctrl = OmniRobotController()
        try:
            await ctrl.init_handles(sim, robot_name=model)
        except Exception as e:
            print(f"⚠️ Skipping {rid} ({model}): {e}")
            continue

        pos = await ctrl.get_position()
        dist = math.hypot(goal_pos[0] - pos[0], goal_pos[1] - pos[1])
        candidates.append((rid, pos, dist))

    if not candidates:
        print("⛔ No available idle robots.")
        return None, None

    robot_id, start_pos, _ = min(candidates, key=lambda x: x[2])
    print(f"✅ Nearest available robot: {robot_id} at {start_pos}")
    return robot_id, start_pos


async def excute(sim, start_pos, start_ori, robot_name, goal_pos):
    """
    Execute a mission for the chosen robot_id ('Rob0'/'Rob1'/'Rob2').
    """
    model = ID_TO_MODEL.get(robot_name)
    if not model:
        print(f"⚠️ Unknown robot name: {robot_name}")
        return "FAILED"

    controller = OmniRobotController()
    await controller.init_handles(sim, robot_name=model)

    shared.robot_start = start_pos
    shared.robot_orientation[robot_name] = start_ori
    shared.robot_name = robot_name
    shared.robot_goal[robot_name] = tuple(goal_pos)
    shared.robot_status[robot_name] = "busy"

    path_in_meters = await plan_path(start_pos, goal_pos, robot_name)
    motion = OmniRobotMotion.RobotMotion(sim, controller.wheels)

    if not path_in_meters:
        await motion.stop()
        shared.robot_status[robot_name] = "idle"
        print("❌ Path planning failed for assigned robot.")
        return "FAILED"

    executor = PathExecutor(sim, robot_name, controller.wheels, controller.robot)
    result = await executor.follow_path(path_in_meters)

    if result == "replanned":
        # Start again from current pose, with updated memory map
        new_start = await controller.get_position()
        outcome = await excute(sim, new_start, start_ori, robot_name, goal_pos)
        return outcome if outcome in ("DONE", "FAILED") else "FAILED"

    if result == "DONE":
        print("✅ Path completed successfully.")
        await motion.stop()
        shared.robot_status[robot_name] = "idle"
        print(f"🏁 {robot_name} finished at {time()}")
        return "DONE"

    print("❌ Path following failed.")
    await motion.stop()
    shared.robot_status[robot_name] = "idle"
    print(f"🏁 {robot_name} finished at {time()}")
    return "FAILED"