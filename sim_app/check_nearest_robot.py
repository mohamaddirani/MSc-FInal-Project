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

async def select_and_execute_nearest_robot(sim, goal_pos):

    
    controller0 = OmniRobotController()
    await controller0.init_handles(sim, robot_name='Omnirob0')

    controller1 = OmniRobotController()
    await controller1.init_handles(sim, robot_name='Omnirob1')

    goal_pos = shared.robot_goal
    print(f"📌 Shared goal: {goal_pos}")

    pos0 = await controller0.get_position()
    pos1 = await controller1.get_position()

    dist0 = math.hypot(goal_pos[0] - pos0[0], goal_pos[1] - pos0[1])
    dist1 = math.hypot(goal_pos[0] - pos1[0], goal_pos[1] - pos1[1])
    print(f"Distance for Robot0 is {dist0}, Distance For RObot 1 is {dist1}")
    if dist0 <= dist1:
        assigned_controller = controller0
        start_pos = pos0
        print("🤖 Robot0 assigned to goal")
    else:
        assigned_controller = controller1
        start_pos = pos1
        print("🤖 Robot1 assigned to goal")
    if math.isclose(start_pos[0], pos0[0], abs_tol=1e-4) and math.isclose(start_pos[1], pos0[1], abs_tol=1e-4):
        robot_name = "Rob0"
    else:
        robot_name = "Rob1"
        
    path_in_meters = await plan_path(start_pos, goal_pos, robot_name)
    motion = OmniRobotMotion.RobotMotion(sim, assigned_controller.wheels)
    if not path_in_meters:
        await motion.stop()  # clean stop if plan fails
        print("❌ Path planning failed for assigned robot.")
        return "FAILED"
    executor = PathExecutor(sim, assigned_controller.robot, assigned_controller.wheels)
    result = await executor.follow_path(path_in_meters)
    print(f"🤖 Initial execution result: {result}")
    replanning_result = None
    if result == "replanned":
        print("🔄 Path replanned due to obstacle.")
        start_pos = await assigned_controller.get_position()
        path_in_meters = await plan_path(start_pos, goal_pos, robot_name)
        if not path_in_meters:
            await motion.stop()
            print("❌ Path planning failed for assigned robot.")
            return "FAILED"
        executor = PathExecutor(sim, assigned_controller.robot, assigned_controller.wheels)
        replanning_result = await executor.follow_path(path_in_meters)
        print(f"🤖 Replanned execution result: {replanning_result}")

        if replanning_result == "DONE":
            print("✅ Path completed successfully after replanning.")
            await motion.stop()
            return "DONE"
        elif replanning_result == "FAILED":
            print("❌ Path following failed after replanning.")
            await motion.stop()
            return "FAILED"
        elif replanning_result == "replanned":
            print("🔄 Path replanned again; continuing execution loop...")
            return "replanned"

    # andle original result if there was no replanning
    if result == "DONE":
        print("✅ Path completed successfully.")
        await motion.stop()
        return "DONE"
    elif result == "FAILED":
        print("❌ Path following failed.")
        await motion.stop()
        return "FAILED"
    elif result == "replanned":
        print("🔄 Path replanned; continuing execution loop...")
        return "replanned"

    # fallback
    print(f"⚠️ Unexpected execution result: {result}")
    await motion.stop()
    return "FAILED"
