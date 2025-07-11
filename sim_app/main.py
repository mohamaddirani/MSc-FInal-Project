# sim_app/main.py
import asyncio
from sim_app.sim_client import get_sim
from sim_app.sensor_fetch import fetch_sensor_data
from sim_app import shared
from sim_app.check_nearest_robot import select_and_execute_nearest_robot
from sim_app.plotter import plot_astar_path, plot_sensor_data, plot_executed_path, plot_planned_path
from sim_app.shared import latest_astar_path, executed_path, planned_path



async def wait_for_goal():
    print("⏳ Waiting for goal from LLM...")
    while shared.robot_goal is None:
        await asyncio.sleep(0.5)
    print(f"✅ Goal received: {shared.robot_goal}")

async def run():
    """
    Asynchronously runs the main simulation loop for robot path planning and execution.
    This function waits for a goal to be set, connects to the CoppeliaSim simulation environment,
    and starts the simulation. In a continuous loop, it fetches sensor data for multiple robots,
    selects and commands the nearest robot to execute a path toward the goal, and generates plots
    for sensor data and paths. The loop handles replanning if obstacles are detected and terminates
    when the robot reaches the goal or fails. Handles graceful shutdown on user interruption.
    Raises:
        KeyboardInterrupt: If the user interrupts the execution.
    """
    await wait_for_goal()

    client, sim = await get_sim()
    print("✅ Connected to CoppeliaSim")

    await sim.startSimulation()

    try:
        while True:
            # Always get latest sensors before planning
            
            await fetch_sensor_data(sim, "Rob0_S300_sensor1")
            await fetch_sensor_data(sim, "Rob0_S300_sensor2")
            await fetch_sensor_data(sim, "Rob0_S3001_sensor1")
            await fetch_sensor_data(sim, "Rob0_S3001_sensor2")
            await fetch_sensor_data(sim, "Rob1_S300_sensor1")
            await fetch_sensor_data(sim, "Rob1_S300_sensor2")
            await fetch_sensor_data(sim, "Rob1_S3001_sensor1")
            await fetch_sensor_data(sim, "Rob1_S3001_sensor2")

            goal_pos = shared.robot_goal
            result = await select_and_execute_nearest_robot(sim, goal_pos)
            print(f"🤖 Selected robot execution result: {result}")

            if shared.robot_name:
                plot_sensor_data(shared.robot_name, filename=f"sensor_data_{shared.robot_name}.png")
                plot_astar_path(
                    path=latest_astar_path,
                    start=shared.robot_start,
                    goal=shared.robot_goal,
                    filename=f"astar_path_{shared.robot_name}.png"
                )
                plot_executed_path(
                    path=executed_path,
                    start=shared.robot_start,
                    goal=shared.robot_goal,
                    filename=f"executed_path_{shared.robot_name}.png"
                )
                plot_planned_path(
                    path=planned_path,
                    start=shared.robot_start,
                    goal=shared.robot_goal,
                    filename=f"planned_path_{shared.robot_name}.png"
                )


            if result == "replanned":
                print("🔄 Obstacle triggered replan. Regenerating path...")
                await asyncio.sleep(0.5)
                continue
            elif result in ["DONE", "FAILED"]:
                print(f"🏁 Final outcome: {result}")
                shared.robot_goal = None  # Clear shared goal
                break

            
    except KeyboardInterrupt:
        print("🛑 Stopped by user.")
        await client.__aexit__(None, None, None)
