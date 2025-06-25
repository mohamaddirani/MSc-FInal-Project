# sim_app/main.py

import asyncio
import matplotlib.pyplot as plt
from sim_app.sim_client import get_sim
from sim_app.sensor_fetch import fetch_sensor_data
from sim_app.plotter import plot_lidar
from sim_app.robot_controller import OmniRobotController

GOAL = [6.3, 5.7]

async def run():
    client, sim = await get_sim()
    print("✅ Connected to CoppeliaSim")

    controller = OmniRobotController()
    await controller.init_handles(sim)

    await sim.startSimulation()

    #plt.ion()
    #plt.figure()

    try:
        while True:
            await fetch_sensor_data(sim, "S300_sensor1")  # back
            await fetch_sensor_data(sim, "S300_sensor2")  # left
            await fetch_sensor_data(sim, "S3001_sensor1") # front
            await fetch_sensor_data(sim, "S3001_sensor2") # right
            plot_lidar()

            reached = await controller.move_to_goal(GOAL)
            if reached:
                print("🎯 Goal Reached!")
                break

            await asyncio.sleep(0.2)
    except KeyboardInterrupt:
        print("🛑 Stopped by user.")
    finally:
        await controller.stop()
        #await sim.stopSimulation()
        #plt.ioff()
        #plt.show()
        await client.__aexit__(None, None, None)
