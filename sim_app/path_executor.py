# sim_app/path_executor.py

import asyncio
import math
from sim_app.obstacle_awareness import is_path_clear, check_sensors_for_obstacle
from sim_app.sensor_fetch import fetch_sensor_data
from sim_app import shared
from sim_app.robot_motion import RobotMotion
from sim_app.robot_motion import SPEED

goal_error_threshold = 0.1
side_obstacle_threshold = 1.2
vertical_obstacle_threshold = 0.74

class PathExecutor:
    """
    Executes a planned path for a robot in a simulation environment, handling movement, obstacle avoidance, and goal alignment.
    Attributes:
        sim: The simulation environment object.
        motion: An instance of RobotMotion for controlling robot movement.
        robot: The robot object or identifier.
        cleared (str): Flag indicating if an obstacle has been cleared ("True"/"False").
        Failed (str): Flag indicating if path execution has failed ("True"/"False").
    Methods:
        __init__(sim, robot, wheels):
            Initializes the PathExecutor with the simulation, robot, and wheels.
        async get_position():
            Retrieves the current (x, y) position of the robot.
        async follow_path(path):
            Follows a given path (list of waypoints), moving the robot to each goal while handling obstacles and alignment.
            Returns "DONE" if the path is completed, "FAILED" if execution fails, or "replanned" if replanning is triggered.
        async move_to_goal(goal):
            Moves the robot towards a specified goal, checking for obstacles and alignment.
            Returns "MOVING" if still in progress, "DONE" if the goal is reached, "FAILED" if blocked, or "replanned" if replanning is needed.
        async check_alignment(direction, coordinates_for_goal):
            Checks if the robot is aligned with the goal along the X or Y axis based on obstacle direction and goal coordinates.
            Returns "X-AXIS ALIGNED", "Y-AXIS ALIGNED", or None.
        async handle_obstacle(Robot, direction, goal, dx, dy):
            Handles obstacle avoidance by moving towards a temporary goal to bypass the obstacle.
            Updates the 'cleared' and 'Failed' flags based on the outcome.
    """

    def __init__(self, sim, robot, wheels):
        self.sim = sim
        self.motion = RobotMotion(sim, wheels)
        self.robot = robot
        self.cleared = "False"  # Flag to indicate if obstacle is cleared
        self.Failed = "False"
    async def get_position(self):
        pos = await self.sim.getObjectPosition(self.robot, -1)
        return pos[:2]

    async def follow_path(self, path):
        i = 0
        for goal in path[1:]:
            #print(f"path: {path}")
            shared.planned_path.append(goal)
            while True:
                i += 1
                print(f"Moving try: {i}")
                print(f"🔁 About to move to goal: {goal}")
                
                result = await self.move_to_goal(goal)
                print(f"✅ move_to_goal returned: {result}")
                
                coordinates_for_goal = (shared.robot_goal[0] - goal[0], shared.robot_goal[1] - goal[1])
                print(f"Result of move_to_goal: {result}")
                if result == "replanned" or result == "FAILED" or result == "DONE":
                    print(f"🚨 follow_path(): exiting loop with result: {result}")
                    await asyncio.sleep(0.1)
                    return "replanned" if result == "replanned" else "FAILED" if result == "FAILED" else "DONE"
                elif result == "MOVING":
                    if i >= len(path) and (abs(coordinates_for_goal[0]) > goal_error_threshold or abs(coordinates_for_goal[1]) > goal_error_threshold):
                        print("Reached the end of the path.")
                        return "FAILED"
                    await asyncio.sleep(0.1)
                    break
                
        print("✅ Completed all waypoints successfully.")
        return "DONE"


    async def move_to_goal(self, goal):
        current = await self.get_position()
        shared.executed_path.append(current)
        dx = goal[0] - current[0]
        dy = goal[1] - current[1]
        dist = math.hypot(dx, dy)
        coordinates_for_goal = (shared.robot_goal[0] - current[0], shared.robot_goal[1] - current[1])
        
        dx = 0 if abs(dx) < goal_error_threshold else dx
        dy = 0 if abs(dy) < goal_error_threshold else dy
        print(f"Moving to goal: {goal}, current position: {current}, dx: {dx}, dy: {dy}, dist: {dist}")

        if abs(coordinates_for_goal[0]) < goal_error_threshold and abs(coordinates_for_goal[1]) < goal_error_threshold:
            print(f"✅ Goal reached at coordinates: {coordinates_for_goal}")
            vx = vy = 0
            return "DONE"
        elif abs(dy) < goal_error_threshold:
            print(f"dx: {dx}, dy: {dy} based on abs(dy) < goal_error_threshold")
            await asyncio.sleep(0.1)
            vx = SPEED * (dx / dist)
            vy = 0
            await self.motion.set_velocity(vx, vy)
        elif abs(dx) < goal_error_threshold:
            print(f"dx: {dx}, dy: {dy} based on abs(dx) < goal_error_threshold")
            await asyncio.sleep(0.1)
            vx = 0
            vy = SPEED * (dy / dist)
            await self.motion.set_velocity(vx, vy)
        else:
            print(f"both dx: {dx}, dy: {dy} are significant")
            vx = SPEED * (dx / dist)
            vy = SPEED * (dy / dist)
            await self.motion.set_velocity(vx, vy)
        back_dist = left_dist = front_dist = right_dist = float('inf')
        print(f"self.robot: {self.robot}, dx: {dx}, dy: {dy}, vx: {vx}, vy: {vy}")
        if self.robot == 64:
            Robot = "Rob0"
        elif self.robot == 105:
            Robot = "Rob1"
        s01 = await fetch_sensor_data(self.sim, f"{Robot}_S300_sensor1")
        s02 = await fetch_sensor_data(self.sim, f"{Robot}_S300_sensor2")
        s11 = await fetch_sensor_data(self.sim, f"{Robot}_S3001_sensor1")
        s12 = await fetch_sensor_data(self.sim, f"{Robot}_S3001_sensor2")
        print(f"Sensor data fetched for {Robot}:\ns01={s01}\ns02={s02}\ns11={s11}\ns12={s12}")
        if s01 is not None:
            back_x, back_y, back_dist = s01
        if s02 is not None:
            left_x, left_y, left_dist = s02
        if s11 is not None:
            front_x, front_y, front_dist = s11
        if s12 is not None:
            right_x, right_y, right_dist = s12
        
        obstacles = check_sensors_for_obstacle(dx, dy, Robot)
        direction = ["Free","Free"]

        if abs(vx) > 0.05 and abs(vy) > 0.05:
            if obstacles[0] == "left" and dx > 0 and obstacles[1] == "back" and dy > 0:
                direction = obstacles  # X direction
            elif obstacles[0] == "right" and dx < 0 and obstacles[1] == "back" and dy > 0:
                direction = obstacles  # X direction
            elif obstacles[0] == "left" and dx > 0 and obstacles[1] == "front" and dy < 0:
                direction = obstacles
            elif obstacles[0] == "right" and dx < 0 and obstacles[1] == "front" and dy < 0:
                direction = obstacles

            elif obstacles[0] == "left" and dx < 0 and obstacles[1] == "back" and dy > 0:
                direction = ["Free", obstacles[1]]  # X direction
            elif obstacles[0] == "right" and dx > 0 and obstacles[1] == "back" and dy > 0:
                direction = ["Free", obstacles[1]]  # X direction
            elif obstacles[0] == "left" and dx > 0 and obstacles[1] == "front" and dy > 0:
                direction = [obstacles[0], "Free"]
            elif obstacles[0] == "right" and dx < 0 and obstacles[1] == "front" and dy > 0:
                direction = [obstacles[0], "Free"] 

            elif obstacles[0] == "Free" and obstacles[1] == "back" and dy > 0:
                direction = ["Free", obstacles[1]]  # X direction
            elif obstacles[0] == "Free" and obstacles[1] == "back" and dy > 0:
                direction = ["Free", obstacles[1]]  # X direction
            elif obstacles[0] == "left" and dx > 0 and obstacles[1] == "Free":
                direction = [obstacles[0], "Free"]
            elif obstacles[0] == "right" and dx < 0 and obstacles[1] == "Free":
                direction = [obstacles[0], "Free"] 


        elif abs(vy) < 0.05 or abs(vx) < 0.05:
            if abs(vy) < 0.05:
                if obstacles[0] == "right" and dx < 0:
                    direction = [obstacles[0], "Free"]  # Y direction
                elif obstacles[0] == "left" and dx > 0:
                    direction = [obstacles[0], "Free"]  # Y direction
            if abs(vx) < 0.05:
                if obstacles[1] == "front" and dy < 0:
                    direction = ["Free" , obstacles[1]]  # Y direction
                elif obstacles[1] == "back" and dy > 0:
                    direction = ["Free" , obstacles[1]]  # Y direction
        print(f"direction : {direction}")

        alignment = await self.check_alignment(obstacles, coordinates_for_goal)
        print(f"alignment: {alignment}")
        if (vx == 0 and vy == 0) or direction != ["Free","Free"]:
            print("entered")
            if not is_path_clear(direction[0] , Robot) and not is_path_clear(direction[1] , Robot):
                print(f"🚧 Obstacle detected in both directions: {direction[0]} and {direction[1]}.")
                await self.motion.stop()
                return "FAILED"
            elif alignment == "X-AXIS ALIGNED" or alignment == "Y-AXIS ALIGNED":
                print(f"✅ Robot is {alignment} at coordinates: {coordinates_for_goal}")
                await self.motion.stop() 
                print(f"‼️ Returning FAILED because robot is aligned: {alignment}")
                return "FAILED"
            elif direction != ["Free" ,"Free"]:
                if direction[0] != "Free":
                    await asyncio.sleep(0.1)
                    await self.motion.stop()
                    await self.handle_obstacle(Robot, direction[0], shared.robot_goal, dx, dy)
                    if self.Failed == "True":
                        #print("🔁 Replanning complete. Resuming original goal.")
                        return "FAILED"  # Continue main loop after obstacle is cleared
                    elif self.cleared == "True":
                        print("🔁 Replanning complete. Resuming original goal.")
                        return "replanned"
                elif direction[1] != "Free":
                    await asyncio.sleep(0.1)
                    await self.motion.stop()
                    await self.handle_obstacle(Robot, direction[1], shared.robot_goal,  dx, dy)
                    if self.Failed == "True":
                        #print("🔁 Replanning complete. Resuming original goal.")
                        return "FAILED"  # Continue main loop after obstacle is cleared
                    elif self.cleared == "True":
                        print("🔁 Replanning complete. Resuming original goal.")
                        return "replanned"
                
        # Compute wheel velocities
        return "MOVING"  # Continue moving towards goal

    async def check_alignment(self, direction, coordinates_for_goal):

            print(f"Checking alignment for direction: {direction}, coordinates_for_goal: {coordinates_for_goal}")

            if direction[1] in ["front", "back"] and abs(coordinates_for_goal[0]) < goal_error_threshold:
                if direction[1] == "front" and coordinates_for_goal[1] > 0:
                    return "X-AXIS ALIGNED"
                elif direction[1] == "back" and coordinates_for_goal[1] < 0:
                    return "X-AXIS ALIGNED"
                else:
                    return None
                
            elif direction[0] in ["left", "right"] and abs(coordinates_for_goal[1]) < goal_error_threshold:
                if direction[0] == "left" and coordinates_for_goal[0] > 0:
                    return "Y-AXIS ALIGNED"
                elif direction[0] == "right" and coordinates_for_goal[0] < 0:
                    return "Y-AXIS ALIGNED"
                else:
                    return None
            else:
                return None

    async def handle_obstacle(self, Robot, direction, goal, dx, dy):
            self.cleared = "False"  # Reset flag for obstacle handling
            current_pos = await self.get_position()
            temp_goal = None
            if direction in ["left", "right"]:
                temp_goal = (current_pos[0], goal[1])
            elif direction in ["front", "back"]:
                temp_goal = (goal[0], current_pos[1])
            a=0
            # Move to temp goal until obstacle clears
            # if(dx == 0 and dy == 0):
            #     return "FAILED"
            # else:
            while a < 25:
                new_direction = check_sensors_for_obstacle(dx, dy, Robot)
                print(f"tries: {a}")
                new_current_pos = await self.get_position()
                new_coordinates_for_goal = [(shared.robot_goal[0] - new_current_pos[0]) , (shared.robot_goal[1] - new_current_pos[1])]
                print(f"new_direction[0]: {new_direction[0]}, new_direction[1]: {new_direction[1]}, direction: {direction}")
                print(f"coordinates_for_goal[0] = {new_coordinates_for_goal[0]}, coordinates_for_goal[0] = {new_coordinates_for_goal[1]}")
                if(abs(new_coordinates_for_goal[0]) <= 0.08 or abs(new_coordinates_for_goal[1]) <= 0.08):
                    a = 25
                    print(f"a = {a}")
                    self.Failed = "True"
                    return "FAILED"
                elif new_direction[0] != direction and new_direction[1] != direction:
                    a = 25
                    self.cleared = "True"
                    break
                await self.move_to_goal(temp_goal)  # Keep moving to temp goal
                await asyncio.sleep(0.01)
                a += 1