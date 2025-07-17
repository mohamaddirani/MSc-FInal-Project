#sim_app/check_nearest_robot.py
import asyncio
import math
from sim_app.robot_controller import OmniRobotController
from sim_app import shared
from sim_app.astar_env import AStarEnvironment , grid_to_meters, meters_to_grid
from sim_app.path_executor import PathExecutor
from sim_app.map_builder import build_occupancy_grid
from sim_app.astar import AStar
import sim_app.robot_motion as OmniRobotMotion
from time import time
import time


async def plan_path(start_pos, goal_pos, robot_name):
    """
    Plan an A* path from start_pos to goal_pos in meters.
    Returns a list of waypoints in meters if successful, otherwise None.
    """
    start_grid = meters_to_grid(start_pos[0], start_pos[1])
    goal_grid = meters_to_grid(goal_pos[0], goal_pos[1])
    

    print(f"📌 Planning path: Start Grid {start_grid}, Goal Grid {goal_grid}")
    grid = build_occupancy_grid(robot_name, grid_size=200, cell_resolution=0.2)  # fresh occupancy grid
    env = AStarEnvironment(grid, start_grid, goal_grid)
    path_finder = AStar(env)
    path = path_finder.search("robot")

    if not path:
        print("❌ No path found during planning!")
        return None

    path_in_meters = [grid_to_meters(*p) for p in path]
    print(f"🚦 A* planned path with {len(path_in_meters)} waypoints.")
    return path_in_meters

async def find_available_robot(sim, goal_pos):
    """
    Selects the nearest available (idle) robot to the given goal_pos.
    Returns (robot_id, start_position) or (None, None) if no robot is free.
    """
    candidates = []

    # Setup robot controllers
    controller0 = OmniRobotController()
    await controller0.init_handles(sim, robot_name='Omnirob0')

    controller1 = OmniRobotController()
    await controller1.init_handles(sim, robot_name='Omnirob1')

    # Check if Rob0 is idle
    if shared.robot_status["Rob0"] == "idle":
        pos0 = await controller0.get_position()
        dist0 = math.hypot(goal_pos[0] - pos0[0], goal_pos[1] - pos0[1])
        candidates.append(("Rob0", pos0, dist0))

    # Check if Rob1 is idle
    if shared.robot_status["Rob1"] == "idle":
        pos1 = await controller1.get_position()
        dist1 = math.hypot(goal_pos[0] - pos1[0], goal_pos[1] - pos1[1])
        candidates.append(("Rob1", pos1, dist1))

    # If no idle robots
    if not candidates:
        print("⛔ No available idle robots.")
        return None, None

    # Sort by distance and select nearest
    selected = sorted(candidates, key=lambda x: x[2])[0]
    robot_id, start_pos, _ = selected
    print(f"✅ Nearest available robot: {robot_id} at {start_pos}")
    return robot_id, start_pos

async def excute(sim, start_pos, robot_name, goal_pos):
    # Choose and initialize the correct controller
    if robot_name == "Rob0":
        controller = OmniRobotController()
        await controller.init_handles(sim, robot_name="Omnirob0")
    elif robot_name == "Rob1":
        controller = OmniRobotController()
        await controller.init_handles(sim, robot_name="Omnirob1")
    else:
        print(f"⚠️ Unknown robot name: {robot_name}")
        return "FAILED"

    shared.robot_start = start_pos
    shared.robot_name = robot_name

    path_in_meters = await plan_path(start_pos, goal_pos, robot_name)
    motion = OmniRobotMotion.RobotMotion(sim, controller.wheels)

    if not path_in_meters:
        await motion.stop()
        print("❌ Path planning failed for assigned robot.")
        return "FAILED"

    executor = PathExecutor(sim, robot_name, controller.wheels, controller.robot)

    result = await executor.follow_path(path_in_meters)

    if result == "replanned":
        start_pos = await controller.get_position()
        outcome = await excute(sim, start_pos, robot_name, goal_pos)
        if outcome in ["DONE","FAILED"]:
            return outcome
              
    elif result == "DONE":
        print("✅ Path completed successfully.")
        await motion.stop()
        print(f"🏁 {robot_name} finished at {time.time()}")
        outcome = "DONE"
        return outcome
    
    elif result == "FAILED":
        print("❌ Path following failed.")
        await motion.stop()
        print(f"🏁 {robot_name} finished at {time.time()}")
        outcome = "FAILED"
        return outcome
