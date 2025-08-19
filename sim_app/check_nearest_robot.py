import math
import numpy as np
from time import time

from sim_app.robot_controller import OmniRobotController
from sim_app import shared
from sim_app.astar_env import AStarEnvironment, grid_to_meters, meters_to_grid
from sim_app.path_executor import PathExecutor
from sim_app.astar import AStar
import sim_app.robot_motion as OmniRobotMotion
from sim_app.map_builder import get_planning_costmap


ROBOT_IDS = ["Rob0", "Rob1", "Rob2"]
ID_TO_MODEL = {"Rob0": "Omnirob0", "Rob1": "Omnirob1", "Rob2": "Omnirob2"}

# Tiny launch/arrival carve (cells)
CLEAR_START_GOAL_RADIUS = 1

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



async def plan_path(start_pos, goal_pos, robot_name):
    if not shared.FREEZE_MAP:
        shared.ensure_map_covers([start_pos[0], goal_pos[0]],
                                 [start_pos[1], goal_pos[1]], margin_m=1.0)

    grid = _current_planning_grid()
    if grid.size == 0:
        print("⛔ Planning grid is empty. Did you load the saved map?")
        return None, None

    res  = shared.MAP_RESOLUTION
    start_grid = meters_to_grid(start_pos[0], start_pos[1], grid, res)
    goal_grid  = meters_to_grid(goal_pos[0],  goal_pos[1],  grid, res)

    H, W = grid.shape
    for (gx, gy), tag in ((start_grid, "start"), (goal_grid, "goal")):
        if not (0 <= gx < W and 0 <= gy < H):
            print(f"⛔ {tag} grid {gx,gy} lies outside the loaded map {W}x{H}.")
            return None, None

    # Optional: carve tiny disk at start/goal to avoid first-step detour
    def _clear_disk_inplace(g, gx, gy, r=1):
        if not (0 <= gx < g.shape[1] and 0 <= gy < g.shape[0]): return
        y0 = max(0, gy-r); y1 = min(g.shape[0], gy+r+1)
        x0 = max(0, gx-r); x1 = min(g.shape[1], gx+r+1)
        rr = r*r
        for y in range(y0, y1):
            dy2 = (y-gy)*(y-gy)
            row = g[y]
            for x in range(x0, x1):
                dx = x-gx
                if dx*dx + dy2 <= rr:
                    row[x] = 0.0

    if grid[start_grid[1], start_grid[0]] >= 0.99:
        _clear_disk_inplace(grid, start_grid[0], start_grid[1], r=1)
    if grid[goal_grid[1], goal_grid[0]] >= 0.99:
        _clear_disk_inplace(grid, goal_grid[0], goal_grid[1], r=1)

    env = AStarEnvironment(
        grid,
        start_grid,
        goal_grid,
        res,
        block_threshold=0.99,
        soft_cost_gain=0.5
    )
    path_g = AStar(env).search("robot")
    if not path_g:
        print("❌ No path found during planning!")
        return None, None

    path_m = [grid_to_meters(x, y, grid, res) for (x, y) in path_g]
    print(f"🚦 A* planned path with {len(path_m)} waypoints.")
    return path_m, grid

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


async def excute(sim, start_pos, start_ori, robot_name, goal_pos):
    model = ID_TO_MODEL.get(robot_name)
    if not model:
        print(f"⚠️ Unknown robot name: {robot_name}")
        return "FAILED"

    controller = OmniRobotController(sim)
    await controller.init_handles(sim, robot_name=model)

    
    shared.robot_name = robot_name
    shared.robot_start_positions[robot_name] = start_pos
    shared.robot_goal[robot_name] = tuple(goal_pos)
    shared.robot_status[robot_name] = "busy"

    # ---- replace recursion with a small replan loop ----
    max_replans = 8
    for _ in range(max_replans):
        path_in_meters, plan_grid = await plan_path(start_pos, goal_pos, robot_name)
        motion = OmniRobotMotion.RobotMotion(sim, controller.wheels)

        if not path_in_meters:
            await motion.stop()
            shared.robot_status[robot_name] = "idle"
            print("❌ Path planning failed for assigned robot.")
            return "FAILED"

        executor = PathExecutor(sim, robot_name, controller.wheels, controller.robot, plan_grid)
        result = await executor.follow_path(path_in_meters)

        if result == "replanned":
            # get live pose and try again
            start_pos = await controller.get_position()
            continue

        if result == "DONE":
            print("✅ Path completed successfully.")
            await motion.stop()
            shared.robot_status[robot_name] = "idle"
            print(f"🏁 {robot_name} finished at {time()}")
            return "DONE"

        # FAILED or something else
        break

    print("❌ Path following failed.")
    await motion.stop()
    shared.robot_status[robot_name] = "idle"
    print(f"🏁 {robot_name} finished at {time()}")
    return "FAILED"
