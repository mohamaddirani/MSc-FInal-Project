# sim_app/robots_awareness.py
import math
import asyncio
from sim_app import shared
import sim_app.robot_motion as OmniRobotMotion
from sim_app.robot_controller import OmniRobotController
from sim_app.obstacle_awareness import is_path_clear, check_sensors_for_obstacle
from sim_app.sensor_fetch import fetch_sensor_data

speed = 0.1
controller = OmniRobotController()

async def find_nearest_free_spot(current_position, block_direction, toward_goal, blocking_robot):
    # Implement logic to find the nearest free spot based on the block_direction
    dx = 0.1 if toward_goal[0] < 0 else -0.1
    dy = 0.1 if toward_goal[1] < 0 else -0.1
    safe_margin = 2 
    if block_direction in ["right", "left"]:
        # Check for free spots in the Y direction
        dx = 0
        free_spot = check_sensors_for_obstacle(dx , dy , blocking_robot)
        if free_spot[1] == "Free":
            return [current_position[0], current_position[1] + safe_margin]
        else:
            free_spot = check_sensors_for_obstacle(dx , -dy , blocking_robot)
            if free_spot[1] == "Free":
                return [current_position[0], current_position[1] + safe_margin]
    elif block_direction in ["front", "back"]:
        # Check for free spots in the X direction
        dy = 0
        free_spot = check_sensors_for_obstacle(dx , dy , blocking_robot)
        if free_spot[0] == "Free":
            return [current_position[0] + safe_margin, current_position[1]]
        else:
            free_spot = check_sensors_for_obstacle(-dx , dy , blocking_robot)
            if free_spot[0] == "Free":
                return [current_position[0] + safe_margin, current_position[1]]
    return None
           
async def is_obstacle_a_robot(sim, block_direction, detected_x, detected_y, active_robot, threshold):
    closest_robot = None

    for robot_name, pos in shared.robot_positions.items():
        if robot_name == active_robot:
            continue

        # Optional: refresh positions from sim
        robot_id = await sim.getObject(f"/Omni{robot_name.lower()}")
        pos = await sim.getObjectPosition(robot_id, -1)
        shared.robot_positions[robot_name] = pos[:2]

        rx, ry = pos[:2]
        dx = detected_x - rx
        dy = detected_y - ry

        if abs(dx) <= threshold and block_direction in ["right", "left"]:
            closest_robot = robot_name
        elif abs(dy) <= threshold and block_direction in ["front", "back"]:
            closest_robot = robot_name

    if closest_robot:
        print(f"🤖 Obstacle is robot: {closest_robot}")
        blocking_robot_ID = await sim.getObject(f"/Omni{closest_robot.lower()}")
        return blocking_robot_ID, closest_robot

    return None  # Static object

async def request_robot_to_clear(sim, blocking_robot_ID, toward_goal, active_robot, blocking_robot, block_direction):
    print(f"⛔ Asking {blocking_robot} to clear path for {active_robot}")
    
    # ⚠️ Correctly create and initialize the controller for the blocking robot
    controller = OmniRobotController()
    await controller.init_handles(sim, f"Omni{blocking_robot.lower()}")
    motion = OmniRobotMotion.RobotMotion(sim, controller.wheels)

    pos = shared.robot_positions[blocking_robot]
    temp_goal = await find_nearest_free_spot(pos, block_direction, toward_goal, blocking_robot)

    if temp_goal:
        print(f"🧭 {blocking_robot} will move temporarily to {temp_goal}")

        while True:
            current = await sim.getObjectPosition(blocking_robot_ID, -1)
            dx = temp_goal[0] - current[0]
            dy = temp_goal[1] - current[1]
            vx = speed * dx
            vy = speed * dy
            await motion.set_velocity(vx, vy)

            shared.robot_positions[blocking_robot] = current[:2]

            # Wait until path is clear again
            await fetch_sensor_data(sim, f"{active_robot}_S300_sensor1")
            await fetch_sensor_data(sim, f"{active_robot}_S300_sensor2")
            await fetch_sensor_data(sim, f"{active_robot}_S3001_sensor1")
            await fetch_sensor_data(sim, f"{active_robot}_S3001_sensor2")
            obstacle = check_sensors_for_obstacle(temp_goal[0], temp_goal[1], active_robot)

            if block_direction != obstacle[0] and block_direction != obstacle[1]:
                await motion.stop()
                shared.robot_status[blocking_robot] = "idle"
                return True
            await asyncio.sleep(0.1)
    return False


async def get_position(sim,ID):
    pos = await sim.getObjectPosition(ID, -1)
    return pos[:2]
