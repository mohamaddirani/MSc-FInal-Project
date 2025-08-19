import asyncio, math, numpy as np

from sim_app.obstacle_awareness import is_path_clear, check_sensors_for_obstacle
from sim_app.sensor_fetch import fetch_sensor_data
from sim_app import shared
from sim_app.robot_motion import RobotMotion
from sim_app.map_builder import update_memory_with_latest, rebuild_costmap
from sim_app.astar_env import AStarEnvironment, meters_to_grid, grid_to_meters
from sim_app.astar import AStar

goal_error_threshold = 0.1
speed = 100*math.pi/180
MAP_RES = shared.MAP_RESOLUTION

# helpers (top of file)
def _wrap_angle(a):
    while a > math.pi:  a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def world_to_body(vx_w, vy_w, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    vx_b =  c*vx_w + s*vy_w
    vy_b = -s*vx_w + c*vy_w
    return vx_b, vy_b


class PathExecutor:
    def __init__(self, sim, robot, wheels, robot_ID):
        self.sim = sim
        self.wheels = wheels
        self.robot = robot
        self.ID = robot_ID

        self.yaw_ref = None
        self.k_yaw = 1.5          # small P gain
        self.omega_max = 0.6      # rad/s clamp
    
    async def get_position(self):
        pos = await self.sim.getObjectPosition(self.ID, -1)
        return pos[:2]
    async def get_orientation(self):
        ori = await self.sim.getObjectOrientation(self.ID, -1)
        return ori[:3]

    async def reset_orientation(self):
        ori = shared.robot_orientation[self.robot]
        await self.sim.setObjectOrientation(self.ID, -1, ori)

    async def follow_path(self, path):
        
        for waypoint in path[2:]:
            await self.reset_orientation()
            shared.planned_path.append(waypoint)
            while True:
                if self.yaw_ref is None:
                    self.yaw_ref = (await self.get_orientation())[2]
                result = await self.move_to_goal(waypoint)
                if result is not None:
                    if result == "MOVING":
                        print(f"reached waypoint {waypoint}")
                        break
                    else:
                        return result
        return "DONE"

    async def move_to_goal(self, waypoint):
        Robot = self.robot
        motion = RobotMotion(self.sim, self.wheels)
        await self.reset_orientation()
        current = await self.get_position()
        current_ori = await self.get_orientation()
        shared.robot_positions[Robot] = tuple(current)
        shared.executed_path.append(current)
        dx = waypoint[0] - current[0]
        dy = waypoint[1] - current[1]
        dist = math.hypot(dx, dy)
        goal_coords = shared.robot_goal[Robot]
        coordinates_for_goal = (goal_coords[0] - current[0], goal_coords[1] - current[1])

        dx = 0 if abs(dx) < goal_error_threshold else dx
        dy = 0 if abs(dy) < goal_error_threshold else dy

        # --- fetch latest sensors & update global map memory ---
        await fetch_sensor_data(self.sim, f"{Robot}_S300_combined_data")
        await fetch_sensor_data(self.sim, f"{Robot}_S3001_combined_data")
        #await self.reset_orientation()
        shared.robot_orientation[Robot] = await self.get_orientation()
        update_memory_with_latest(Robot)
        rebuild_costmap(inflation_radius_m=0.45)

        # def _replan_from_here():
        #     # make sure the map covers current pose + goal
        #     shared.ensure_map_covers([current[0], goal_coords[0]],
        #                              [current[1], goal_coords[1]], margin_m=1.0)
        #     occ = (shared.global_costmap > 0.0).astype(np.uint8)
        #     start_g = meters_to_grid(current[0], current[1], occ, MAP_RES)
        #     goal_g  = meters_to_grid(goal_coords[0], goal_coords[1], occ, MAP_RES)
        #     env = AStarEnvironment(occ, start_g, goal_g, MAP_RES)
        #     planner = AStar(env)
        #     path_g = planner.search("robot")
        #     if not path_g:
        #         return None
        #     return [grid_to_meters(x, y, occ, MAP_RES) for (x, y) in path_g]

        # --- quick occupancy check at the waypoint cell ---
        occ_bin = (shared.global_occupancy > 0)
        way_g = meters_to_grid(waypoint[0], waypoint[1], occ_bin, MAP_RES)
        wy = min(max(way_g[1], 0), occ_bin.shape[0]-1)
        wx = min(max(way_g[0], 0), occ_bin.shape[1]-1)
        if occ_bin[wy, wx]:
            # new_path = _replan_from_here()
            # if new_path:
            return "replanned"

        align_tol = 0.10  # m

        if abs(dx) < goal_error_threshold and abs(dy) < goal_error_threshold:
            await motion.stop()
            return "MOVING"

        if abs(dx) > align_tol and abs(dy) > align_tol:
            move = "Diagonal"
        else:
            move = "Horizontal" if abs(dx) >= abs(dy) else "Vertical"

        # current yaw
        yaw = (await self.get_orientation())[2]
        shared.robot_orientation[Robot] = (0.0, 0.0, yaw)


        vx , vy = await motion.set_velocity(dx, dy, move) 
        print(f" dx = {dx}, dy = {dy}, vx = {vx}, vy = {vy}, move = {move}")
        # --- local reactive layer (same logic you had) ---
        obstacles = check_sensors_for_obstacle(dx, dy, Robot)
        direction = ["Free","Free"]
        await self.reset_orientation()
        print(f"Obstacle directions: {obstacles}")
        # (keep your directional selection heuristics)
        # if abs(vx) > 0.05 and abs(vy) > 0.05:
        #     if obstacles[0] == "left" and dx > 0 and obstacles[1] == "back" and dy > 0:
        #         direction = obstacles
        #     elif obstacles[0] == "right" and dx < 0 and obstacles[1] == "back" and dy > 0:
        #         direction = obstacles
        #     elif obstacles[0] == "left" and dx > 0 and obstacles[1] == "front" and dy < 0:
        #         direction = obstacles
        #     elif obstacles[0] == "right" and dx < 0 and obstacles[1] == "front" and dy < 0:
        #         direction = obstacles
        #     elif obstacles[0] == "left" and dx < 0 and obstacles[1] == "back" and dy > 0:
        #         direction = ["Free", obstacles[1]]
        #     elif obstacles[0] == "right" and dx > 0 and obstacles[1] == "back" and dy > 0:
        #         direction = ["Free", obstacles[1]]
        #     elif obstacles[0] == "left" and dx > 0 and obstacles[1] == "front" and dy > 0:
        #         direction = [obstacles[0], "Free"]
        #     elif obstacles[0] == "right" and dx < 0 and obstacles[1] == "front" and dy > 0:
        #         direction = [obstacles[0], "Free"]
        #     elif obstacles[0] == "Free" and obstacles[1] == "back" and dy > 0:
        #         direction = ["Free", obstacles[1]]
        #     elif obstacles[0] == "left" and dx > 0 and obstacles[1] == "Free":
        #         direction = [obstacles[0], "Free"]
        #     elif obstacles[0] == "right" and dx < 0 and obstacles[1] == "Free":
        #         direction = [obstacles[0], "Free"]
        # elif abs(vy) < 0.05 or abs(vx) < 0.05:
        #     if abs(vy) < 0.05:
        #         if obstacles[0] == "right" and dx < 0:
        #             direction = [obstacles[0], "Free"]
        #         elif obstacles[0] == "left" and dx > 0:
        #             direction = [obstacles[0], "Free"]
        #         if abs(coordinates_for_goal[0]) < goal_error_threshold:
        #             direction[1] = obstacles[1]
        #     if abs(vx) < 0.05:
        #         if obstacles[1] == "front" and dy < 0:
        #             direction = ["Free" , obstacles[1]]
        #         elif obstacles[1] == "back" and dy > 0:
        #             direction = ["Free" , obstacles[1]]
        #         if abs(coordinates_for_goal[1]) < goal_error_threshold:
        #             direction[0] = obstacles[0]

        alignment = await self.check_alignment(obstacles, coordinates_for_goal)
        if alignment:
            # await motion.stop()
            # new_path = _replan_from_here()
            # if new_path:
                await motion.stop()
                return "replanned"
            # return "FAILED"
        # if alignment == "X-AXIS ALIGNED":
        #     print("Robot is aligned along the X-axis.")
        #     return "FAILED"
        # elif alignment == "Y-AXIS ALIGNED":
        #     print("Robot is aligned along the Y-axis.")
        #     return "FAILED"

        # --- blocked: try to replan first; stop only if no path ---
        x_obs, y_obs = obstacles  # tuple: ('left'|'right'|'Free', 'front'|'back'|'Free')

        def axis_blocked(dx, dy, x_obs, y_obs):
            # Block if obstacle is exactly in the direction of travel
            if dx > 0 and x_obs == 'left':   return True
            if dx < 0 and x_obs == 'right':  return True
            if dy > 0 and y_obs == 'back':   return True
            if dy < 0 and y_obs == 'front':  return True
            return False

        # Replace the current obstacle branch with:
        if (vx == 0 and vy == 0) or obstacles != ("Free","Free"):  # tuple, not list
            if not is_path_clear(obstacles[0], Robot) and not is_path_clear(obstacles[1], Robot):
                await motion.stop()
                # new_path = _replan_from_here()
                # if new_path:
                return "replanned"
            
            else:
                i = 0 if obstacles[0] != "Free" else 1
                await motion.stop()
                handle = await self.handle_obstacle(Robot, obstacles[i], dx, dy)
                if handle == "CLEARED":
                    return "replanned"
                elif handle is not None:
                    return handle
        
        if abs(dx) < goal_error_threshold and abs(dy) < goal_error_threshold:
            return "MOVING"

    async def check_alignment(self, direction, coordinates_for_goal):
        if direction[1] in ["front", "back"] and abs(coordinates_for_goal[0]) < goal_error_threshold:
            if direction[1] == "front" and coordinates_for_goal[1] > 0:
                return "X-AXIS ALIGNED"
            elif direction[1] == "back" and coordinates_for_goal[1] < 0:
                return "X-AXIS ALIGNED"
        elif direction[0] in ["left", "right"] and abs(coordinates_for_goal[1]) < goal_error_threshold:
            if direction[0] == "left" and coordinates_for_goal[0] > 0:
                return "Y-AXIS ALIGNED"
            elif direction[0] == "right" and coordinates_for_goal[0] < 0:
                return "Y-AXIS ALIGNED"

    async def handle_obstacle(self, Robot, direction, dx, dy):
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
