# sim_app/main.py
import os
import json
import asyncio
from sim_app.sim_client import get_sim
from sim_app.sensor_fetch import fetch_sensor_data
from sim_app import shared
from sim_app.check_nearest_robot import excute
from sim_app.plotter import plot_astar_path, plot_sensor_data, plot_executed_path, plot_planned_path
from sim_app.robot_controller import OmniRobotController
from time import time
import time
import asyncio



async def wait_for_any_robot_goal():
    print("⏳ Waiting for any robot goal from LLM...")
    while True:
        try:
            if os.path.exists("shared_goal.json"):
                # Read & load the file content
                with open("shared_goal.json", "r") as f:
                    goal_data = json.load(f)

                # Set the goals in shared memory
                for robot_id, goal in goal_data.items():
                    shared.robot_goal[robot_id] = goal
                    shared.robot_status[robot_id] = "busy"

                robot_id, goal_pos = list(goal_data.items())[0]
                shared.robot_name = robot_id
                shared.robot_goal[robot_id] = goal_pos
                shared.robot_status[robot_id] = "busy"


                # Wait briefly to ensure file is released before deletion
                await asyncio.sleep(0.1)

                # Attempt to delete file safely
                try:
                    os.remove("shared_goal.json")
                except PermissionError:
                    print("⚠️ File still locked, will retry delete shortly...")
                    await asyncio.sleep(0.5)
                    try:
                        os.remove("shared_goal.json")
                    except Exception as e:
                        print(f"❌ Failed to delete shared_goal.json: {e}")
                print(f"✅ Loaded goal: {shared.robot_goal}")
                break
        except Exception as e:
            print(f"⚠️ Error while reading goal file: {e}")
        await asyncio.sleep(0.5)

async def get_robot_position(sim, robot_name):
    controller = OmniRobotController()
    await controller.init_handles(sim, robot_name=f"Omnirob{robot_name[-1]}")
    return await controller.get_position()


async def run():
    
    # Clear any previously saved goal
    if os.path.exists("shared_goal.json"):
        print("🧹 Clearing previous shared goal...")
        with open("shared_goal.json", "w") as f:
            json.dump({
                "Rob0": None,
                "Rob1": None,
                "Rob2": None
            }, f)

    await wait_for_any_robot_goal()

    client, sim = await get_sim()
    print("✅ Connected to CoppeliaSim")

    await sim.startSimulation()

    active_tasks = {}

    try:
       
        while True:
            # Assign tasks for all robots with goals
            for robot_name, goal in shared.robot_goal.items():
                if goal and robot_name not in active_tasks:
                    print(f"🚀 Launching task for {robot_name} at {time.time()}")
                    start_pos = await get_robot_position(sim, robot_name)
                    task = asyncio.create_task(excute(sim, start_pos, robot_name, goal))
                    active_tasks[robot_name] = task
                    
            # Check and handle completed tasks
            done_robots = []
            for robot_name, task in active_tasks.items():
                if task.done():
                    result = task.result()
                    print(f"🏁 Final outcome for {robot_name}: {result}")
                    shared.robot_status[robot_name] = "idle"
                    shared.robot_goal[robot_name] = None
                    done_robots.append(robot_name)

            for robot in done_robots:
                del active_tasks[robot]

            # Wait for more goals if none are active
            if all(goal is None for goal in shared.robot_goal.values()):
                print("🕓 Waiting for new goal...")
                await wait_for_any_robot_goal()

            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        print("🛑 Stopped by user.")
        await client.__aexit__(None, None, None)

if __name__ == "__main__":
    import sys
    import asyncio

    if sys.platform == "win32" and sys.version_info >= (3, 8):
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    asyncio.run(run())

