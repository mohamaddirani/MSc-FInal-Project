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

    def __init__(self, sim, robot, wheels, robot_ID):
        self.sim = sim
        self.motion = RobotMotion(sim, wheels)
        self.robot = robot
        self.ID = robot_ID

    async def get_position(self):
        pos = await self.sim.getObjectPosition(self.ID, -1)
        return pos[:2]
    
    async def reset_orientation(self):
        await self.sim.setObjectOrientation(self.ID, -1, [0, 0, 0])

    async def follow_path(self, path):
        for waypoint in path[1:]:
            shared.planned_path.append(waypoint)
            while True:
                await self.reset_orientation()
                result = await self.move_to_goal(waypoint)
                if result is not None:
                    if result == "MOVING":
                        break
                    else: 
                        return result
        return "DONE"


    async def move_to_goal(self, waypoint):
        Robot = self.robot
        await self.reset_orientation()
        current = await self.get_position()
        shared.executed_path.append(current)
        dx = waypoint[0] - current[0]
        dy = waypoint[1] - current[1]
        dist = math.hypot(dx, dy)
        goal_coords = shared.robot_goal[Robot]
        coordinates_for_goal = (goal_coords[0] - current[0], goal_coords[1] - current[1])
        
        dx = 0 if abs(dx) < goal_error_threshold else dx
        dy = 0 if abs(dy) < goal_error_threshold else dy

        if abs(dy) < goal_error_threshold and abs(dx) < goal_error_threshold:
            return "MOVING"
        elif abs(dy) < goal_error_threshold: #Move only on X
            vx = SPEED * (dx / dist)
            vy = 0
            await self.motion.set_velocity(vx, vy)
        elif abs(dx) < goal_error_threshold: #Move only on Y
            vx = 0
            vy = SPEED * (dy / dist)
            await self.motion.set_velocity(vx, vy)
        else: #Both dx & dy are significant
            print(f"")
            vx = SPEED * (dx / dist)
            vy = SPEED * (dy / dist)
            await self.motion.set_velocity(vx, vy)

        s01 = await fetch_sensor_data(self.sim, f"{Robot}_S300_sensor1")
        s02 = await fetch_sensor_data(self.sim, f"{Robot}_S300_sensor2")
        s11 = await fetch_sensor_data(self.sim, f"{Robot}_S3001_sensor1")
        s12 = await fetch_sensor_data(self.sim, f"{Robot}_S3001_sensor2")

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
                if abs(coordinates_for_goal[0]) < goal_error_threshold:
                    print(abs(coordinates_for_goal[0] - current[0]))
                    direction[1] = obstacles[1]
            if abs(vx) < 0.05:
                if obstacles[1] == "front" and dy < 0:
                    direction = ["Free" , obstacles[1]]  # Y direction
                elif obstacles[1] == "back" and dy > 0:
                    direction = ["Free" , obstacles[1]]  # Y direction
                if abs(coordinates_for_goal[1]) < goal_error_threshold:
                    direction[0] = obstacles[0]

        alignment = await self.check_alignment(obstacles, coordinates_for_goal)

        if (vx == 0 and vy == 0) or direction != ["Free","Free"]:
            if not is_path_clear(direction[0] , Robot) and not is_path_clear(direction[1] , Robot):
                print(f"🚧 Obstacle detected in both directions: {direction[0]} and {direction[1]}.")
                await self.motion.stop()
                return "FAILED"
            elif alignment == "X-AXIS ALIGNED" or alignment == "Y-AXIS ALIGNED":
                await self.motion.stop()
                print(f"‼️ Returning FAILED because robot is aligned: {alignment}")
                return "FAILED"
            else:
                i = 0 if direction[0] != "Free" else 1
                await self.motion.stop()
                handle = await self.handle_obstacle(direction[i], dx, dy)
                if handle == "CLEARED":
                    print("🔁 Replanning complete. Resuming original goal.")
                    return "replanned"
                elif handle is not None:
                    return handle
                    
        if abs(dx) < goal_error_threshold and abs(dy) < goal_error_threshold:
            return "MOVING" #reached desired waypoint       
            
    async def check_alignment(self, direction, coordinates_for_goal):

            #print(f"Checking alignment for direction: {direction}, coordinates_for_goal: {coordinates_for_goal}")
            if direction[1] in ["front", "back"] and abs(coordinates_for_goal[0]) < goal_error_threshold:
                if direction[1] == "front" and coordinates_for_goal[1] > 0:
                    return "X-AXIS ALIGNED"
                elif direction[1] == "back" and coordinates_for_goal[1] < 0:
                    return "X-AXIS ALIGNED"
            
            elif direction[0] in ["left", "right"] and abs(coordinates_for_goal[1]) < goal_error_threshold:
                print(f"enterd")
                if direction[0] == "left" and coordinates_for_goal[0] > 0:
                    return "Y-AXIS ALIGNED"
                elif direction[0] == "right" and coordinates_for_goal[0] < 0:
                    return "Y-AXIS ALIGNED"

    async def handle_obstacle(self, direction, dx, dy):
        Robot = self.robot
        current_pos = await self.get_position()
        robot_goal = shared.robot_goal[Robot]

        if direction in ["left", "right"]:
            temp_goal = (current_pos[0], robot_goal[1])
        elif direction in ["front", "back"]:
            temp_goal = (robot_goal[0], current_pos[1])
        else:
            temp_goal = current_pos

        while True:
            await self.reset_orientation()
            new_direction = check_sensors_for_obstacle(dx, dy, Robot)
            if new_direction[0] != direction and new_direction[1] != direction:
                return "CLEARED"
            move = await self.move_to_goal(temp_goal)
            if move is not None:
                return move
            await asyncio.sleep(0.01)