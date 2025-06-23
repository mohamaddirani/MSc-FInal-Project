# sim_app/main.py

import asyncio
import matplotlib.pyplot as plt
from sim_app.sim_client import get_sim
from sim_app.sensor_fetch import fetch_sensor_data
from sim_app.plotter import plot_lidar
from sim_app.robot_controller import OmniRobotController  # ✅ Correct import

GOAL = [+8.125, +8.150]  # ✅ Your actual goal

async def run():
    client, sim = await get_sim()
    print("✅ Connected to CoppeliaSim")

    controller = OmniRobotController()
    await controller.init_handles(sim)

    await sim.startSimulation()

    plt.ion()
    plt.figure()

    try:
        while True:
            await fetch_sensor_data(sim, "S300")
            await fetch_sensor_data(sim, "S3001")
            plot_lidar()

            reached = await controller.move_to_goal(GOAL)  # ✅ Correct method
            if reached:
                print("🎯 Goal Reached!")
                break

            await asyncio.sleep(0.2)
    except KeyboardInterrupt:
        print("🛑 Stopped by user.")
    finally:
        await controller.stop()  # ✅ Stop wheels cleanly
        await sim.stopSimulation()
        plt.ioff()
        plt.show()
        await client.__aexit__(None, None, None)
