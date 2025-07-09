# sim_app/main.py
import asyncio
import matplotlib.pyplot as plt
from sim_app.sim_client import get_sim
from sim_app.sensor_fetch import fetch_sensor_data
from sim_app.plotter import plot_lidar
from sim_app import shared
from sim_app.check_nearest_robot import select_and_execute_nearest_robot

async def wait_for_goal():
    print("⏳ Waiting for goal from LLM...")
    while shared.robot_goal is None:
        await asyncio.sleep(0.5)
    print(f"✅ Goal received: {shared.robot_goal}")

async def run():
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
