import asyncio, math, numpy as np

from sim_app.obstacle_awareness import is_path_clear, check_sensors_for_obstacle
from sim_app.sensor_fetch import fetch_sensor_data
from sim_app import shared
from sim_app.robot_motion import RobotMotion
from sim_app.map_builder import update_memory_with_latest, rebuild_costmap
from sim_app.astar_env import meters_to_grid
from sim_app.robot_controller import OmniRobotController

goal_error_threshold = 0.1
speed = 100*math.pi/180
MAP_RES = shared.MAP_RESOLUTION



def world_to_body(vx_w, vy_w, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    vx_b =  c*vx_w + s*vy_w
    vy_b = -s*vx_w + c*vy_w
    return vx_b, vy_b


MAP_RES = shared.MAP_RESOLUTION

class PathExecutor:
    def __init__(self, sim, robot_name, wheels, robot_handle, plan_grid):
        self.sim = sim
        self.wheels = wheels
        self.robot = robot_name
        self.ID = robot_handle
        # keep the exact grid used for planning (robots cleared)
        self.plan_grid = plan_grid

    async def get_position(self):
        pos = await self.sim.getObjectPosition(self.ID, -1)
        return pos[:2]

    async def get_orientation(self):
        ori = await self.sim.getObjectOrientation(self.ID, -1)
        return ori[:3]

    async def reset_orientation(self):
        # keep your fixed orientation reset as-is
        await self.sim.setObjectOrientation(
            self.ID, -1,
            [4.235164738711706e-22, 3.394632958307861e-05, -1.4349296282953856e-42]
        )

    async def follow_path(self, path):
        for waypoint in path[2:]:
            await self.reset_orientation()
            shared.planned_path.append(waypoint)
            while True:
                result = await self.move_to_goal(waypoint)
                if result is not None:
                    if result == "MOVING":
                        cur = await self.get_position()
                        print(f"current position: {cur}")
                        print(f"reached waypoint {waypoint}")
                        break
                    else:
                        return result
                # avoid tight CPU spin
                await asyncio.sleep(0.01)
        return "DONE"

    async def move_to_goal(self, waypoint):
        Robot = self.robot
        motion = RobotMotion(self.sim, self.wheels)

        await self.reset_orientation()
        current = await self.get_position()
        shared.robot_positions[Robot] = tuple(current)
        shared.executed_path.append(current)

        dx = waypoint[0] - current[0]
        dy = waypoint[1] - current[1]
        goal_coords = shared.robot_goal[Robot]
        coordinates_for_goal = (goal_coords[0] - current[0], goal_coords[1] - current[1])

        dx = 0 if abs(dx) < goal_error_threshold else dx
        dy = 0 if abs(dy) < goal_error_threshold else dy

        # keep sensors + localization refresh (map updates are gated by FREEZE_MAP)
        await fetch_sensor_data(self.sim, f"{Robot}_S300_combined_data")
        await fetch_sensor_data(self.sim, f"{Robot}_S3001_combined_data")
        shared.robot_orientation[Robot] = await self.get_orientation()
        shared.robot_positions[Robot]   = await self.get_position()
        if not shared.FREEZE_MAP:
            update_memory_with_latest(Robot)
            rebuild_costmap(inflation_radius_m=shared.INFLATION_RADIUS_M)


        # ---- IMPORTANT: check blockage against the SAME planning grid ----
        way_g = meters_to_grid(waypoint[0], waypoint[1], self.plan_grid, MAP_RES)
        wy = min(max(way_g[1], 0), self.plan_grid.shape[0]-1)
        wx = min(max(way_g[0], 0), self.plan_grid.shape[1]-1)
        if self.plan_grid[wy, wx] >= 0.99:
            return "replanned"

        # motion logic unchanged
        align_tol = 0.10
        if abs(dx) < goal_error_threshold and abs(dy) < goal_error_threshold:
            await motion.stop()
            return "MOVING"

        move = "Diagonal" if (abs(dx) > align_tol and abs(dy) > align_tol) else \
               ("Horizontal" if abs(dx) >= abs(dy) else "Vertical")

        vx , vy = await motion.set_velocity(dx, dy, move)
        print(f" dx = {dx}, dy = {dy}, vx = {vx}, vy = {vy}, move = {move}")

        obstacles = check_sensors_for_obstacle(dx, dy, Robot)
        await self.reset_orientation()
        print(f"Obstacle directions: {obstacles}")

        alignment = await self.check_alignment(obstacles, coordinates_for_goal)
        aligned = 0
        if alignment:
            aligned += 1
            print(f"Alignment check passed for {Robot}: {aligned}")
            if aligned < 2:
                await motion.stop()
                return "replanned"
            else:
                return "FAILED"

        if (vx == 0 and vy == 0) or obstacles != ("Free","Free"):
            if not is_path_clear(obstacles[0], Robot) and not is_path_clear(obstacles[1], Robot):
                await motion.stop()
                return "replanned"
            else:
                i = 0 if obstacles[0] != "Free" else 1
                await motion.stop()
                print("obstacle handling")
                handle = await self.handle_obstacle(Robot, obstacles[i], dx, dy)
                if handle == "CLEARED":
                    return "replanned"
                elif handle is not None:
                    return handle

        if abs(dx) < goal_error_threshold and abs(dy) < goal_error_threshold:
            return "MOVING"

    async def check_alignment(self, direction, coordinates_for_goal):
        print("checking alignment")
        if direction[1] in ["front", "back"] and abs(coordinates_for_goal[0]) < goal_error_threshold:
            if (direction[1] == "front" and coordinates_for_goal[1] > 0) or (direction[1] == "back" and coordinates_for_goal[1] < 0):
                return True
        elif direction[0] in ["left", "right"] and abs(coordinates_for_goal[1]) < goal_error_threshold:
            if (direction[0] == "left" and coordinates_for_goal[0] > 0) or (direction[0] == "right" and coordinates_for_goal[0] < 0):
                return True

    async def handle_obstacle(self, Robot, direction, dx, dy):
        aligned = 0
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
            if move is True:
                aligned += 1
                if aligned >= 2:
                    return "FAILED"
            if move is not None:
                return move
            await asyncio.sleep(0.01)
