# sim_app.path_executor.py
from datetime import time
from time import time
import asyncio, math, numpy as np
import os, json
from sim_app.obstacle_awareness import is_path_clear, check_sensors_for_obstacle
from sim_app.sensor_fetch import fetch_sensor_data
from sim_app import shared
from sim_app.robot_motion import RobotMotion
from sim_app.map_builder import update_memory_with_latest, rebuild_costmap
from sim_app.astar_env import meters_to_grid
from sim_app.robot_controller import OmniRobotController
from sim_app.robots_awareness import request_robot_to_clear, return_parked_robot_after_active_done



goal_error_threshold = 0.1
speed = 100*math.pi/180
MAP_RES = shared.MAP_RESOLUTION

CMD_FILE = "shared_cmd.json"     # same as LLM.py
GOAL_FILE = "shared_goal.json"   # LLM may set a new goal here


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
        shared.planned_path = path
        for waypoint in path[2:]:
            await self.reset_orientation()
            while True:
                result = await self.move_to_goal(waypoint)
                if result is not None:
                    if result == "MOVING":
                        
                        break
                    elif result in ["replanned", "FAILED"]:
                        return result
                await asyncio.sleep(0.01)

        await return_parked_robot_after_active_done(self.sim, self.robot)
        return "DONE"


    async def move_to_goal(self, waypoint):
        print(f"Moving to waypoint: {waypoint}")
        Robot = self.robot
        motion = RobotMotion(self.sim, self.wheels)

        # abort path?
        if shared.robot_abort.get(Robot):
            shared.robot_abort[Robot] = False
            await motion.stop()
            return "FAILED"

        await self.reset_orientation()
        current = await self.get_position()
        shared.robot_positions[Robot] = tuple(current)
        shared.executed_path_by_robot[Robot].append(tuple(current))
        shared.latest_astar_path_by_robot[Robot].append(tuple(waypoint))

        dx = waypoint[0] - current[0]
        dy = waypoint[1] - current[1]
        goal_coords = shared.robot_goal[Robot]
        coordinates_for_goal = (goal_coords[0] - current[0], goal_coords[1] - current[1])

        dx = 0 if abs(dx) < goal_error_threshold else dx
        dy = 0 if abs(dy) < goal_error_threshold else dy

        # sensor + optional map refresh
        await fetch_sensor_data(self.sim, f"{Robot}_S300_combined_data")
        await fetch_sensor_data(self.sim, f"{Robot}_S3001_combined_data")
        print("sensor fetched")
        shared.robot_orientation[Robot] = await self.get_orientation()
        shared.robot_positions[Robot]   = await self.get_position()
        if not shared.FREEZE_MAP:
            update_memory_with_latest(Robot)
            rebuild_costmap(inflation_radius_m=shared.INFLATION_RADIUS_M)     # 
        grid_now = getattr(shared, "costmap", None)  # if you keep an inflated costmap
        if grid_now is None:
            grid_now = shared.global_occupancy       # fallback to raw occupancy
        self.plan_grid = grid_now 
        # still block if the target cell itself is hard-blocked in the planning grid
        way_g = meters_to_grid(waypoint[0], waypoint[1], self.plan_grid, MAP_RES)
        wy = min(max(way_g[1], 0), self.plan_grid.shape[0]-1)
        wx = min(max(way_g[0], 0), self.plan_grid.shape[1]-1)
        # treat any non-free (>0) as blocked if you're using an inflated costmap
        if float(self.plan_grid[wy, wx]) >= 0.99:
            return "replanned"

        # reached waypoint?
        align_tol = 0.10
        if abs(dx) < goal_error_threshold and abs(dy) < goal_error_threshold:
            await motion.stop()
            # await asyncio.sleep(1)
            return "MOVING"

        # --- compute intended motion BEFORE obstacle check ---
        move = "Diagonal" if (abs(dx) > align_tol and abs(dy) > align_tol) else \
               ("Horizontal" if abs(dx) >= abs(dy) else "Vertical")
        print("reached checking sensors")
        # --- read obstacles ONCE ---
        dirs, robot_info = check_sensors_for_obstacle(dx, dy, Robot)
        x_dir, y_dir, frdiag, brdiag = dirs
        robot_name_hit, robot_dir_hit = robot_info
        print(f"dirs={dirs}, robot={robot_info}")

        # --- ROBOT obstacle branch ---
        if robot_name_hit:
            blocker = robot_name_hit
            # decide axis & side to request based on the robot direction
            if robot_dir_hit in ["left", "right", "front-left", "back-left", "front-right", "back-right"]:
                # For left/right or diagonal including left/right, clear X
                # Choose side consistent with where we intend to go (dx), fall back to the seen side
                block_direction = ("left" if (dx > 0 or "left" in robot_dir_hit) else "right")
                await motion.stop()
                print(f"Requesting {blocker} to clear path (X-axis, {block_direction})")
                parked = await request_robot_to_clear(self.sim, (goal_coords[0], goal_coords[1]), Robot, blocker, block_direction)
                if parked: return None
                await motion.stop(); return "replanned"

            if robot_dir_hit in ["front", "back"]:
                block_direction = ("front" if (dy < 0 or robot_dir_hit == "front") else "back")
                await motion.stop()
                print(f"Requesting {blocker} to clear path (Y-axis, {block_direction})")
                parked = await request_robot_to_clear(self.sim, (goal_coords[0], goal_coords[1]), Robot, blocker, block_direction)
                if parked: return None
                await motion.stop(); return "replanned"
            

        # --- NON-robot obstacles ---
        # 1) If cardinal blocks exist, use your old logic
        # --- NON-robot obstacles: PAUSE and ask LLM/user ---
        non_robot_blocked = (
            (x_dir != "Free" and abs(dx) > 0) or
            (y_dir != "Free" and abs(dy) > 0) or
            ((frdiag != "Free" or brdiag != "Free") and (abs(dx) > 0 and abs(dy) > 0))
        )
        if non_robot_blocked:
            await motion.stop()
            # choose a representative blocked direction for context
            blocked_dir = x_dir if x_dir != "Free" else (y_dir if y_dir != "Free" else (frdiag if frdiag != "Free" else brdiag))
            decision = await self._await_llm_decision(Robot, blocked_dir)

            if decision == "FAILED":
                await motion.stop()
                return "FAILED"

            if decision == "REPLAN":
                # new goal or go_home will be handled by the main planning loop
                await motion.stop()
                return "replanned"

            if decision in ("CLEARED_AUTO", "CONTINUE"):
                # resume automatic obstacle handler if still blocked
                dirs2, _ = check_sensors_for_obstacle(dx, dy, Robot)
                x2, y2, fr2, br2 = dirs2

                # if still blocked, try your existing detour logic
                if (x2 != "Free" and abs(dx) > 0) or (y2 != "Free" and abs(dy) > 0):
                    block = x2 if x2 != "Free" else y2
                    handle = await self.handle_obstacle(Robot, block, dx, dy)
                    if handle in ("replanned", "FAILED"):
                        return handle
                    if handle == "CLEARED":
                        return "replanned"

                diag2 = fr2 if fr2 != "Free" else (br2 if br2 != "Free" else "Free")
                if diag2 != "Free" and (abs(dx) > 0 and abs(dy) > 0):
                    handle = await self.handle_obstacle(Robot, diag2, dx, dy)
                    if handle in ("replanned", "FAILED"):
                        return handle
                    if handle == "CLEARED":
                        return "replanned"
                # if not blocked anymore, just fall through and continue moving

        else:
            print("No obstacles detected; proceeding.")
                
        await self.reset_orientation()
        # no obstacle: actually move
        vx, vy = await motion.set_velocity(dx, dy, move)
        print(f" dx = {dx}, dy = {dy}, vx = {vx}, vy = {vy}, move = {move}")

    async def handle_obstacle(self, Robot, direction, dx, dy, safety_clear_time: float = 0.6):
        current_pos = await self.get_position()
        robot_goal  = shared.robot_goal[Robot]

        diag = None  # <-- ensure defined

        if direction in ["left", "right"]:
            temp_goal = (current_pos[0], robot_goal[1])
        elif direction in ["front", "back"]:
            temp_goal = (robot_goal[0], current_pos[1])
        elif direction in ["front-left", "front-right"]:
            diag = "fd"
            temp_goal = (robot_goal[0], current_pos[1])
        elif direction in ["back-left", "back-right"]:
            diag = "bd"
            temp_goal = (robot_goal[0], current_pos[1])
        else:
            temp_goal = (robot_goal[0], current_pos[1])  # safe default

        while True:
            await self.reset_orientation()

            # refresh pose each loop (so temp_goal uses fresh coords when we switch axes)
            current_pos = await self.get_position()
            new_dirs, new_rob = check_sensors_for_obstacle(dx, dy, Robot)
            robot_dir = new_rob[1] if new_rob else None

            # cleared if the original 'direction' is not reported in dirs AND not as the robot_dir
            if (direction not in new_dirs) and (robot_dir != direction):
                print(f"waiting before returning")
                await asyncio.sleep(5)
                return "CLEARED"
            
            # for diagonal cases, if we see a left/right component then align on Y instead (your logic)
            if diag and ("right" in new_dirs or "left" in new_dirs):
                temp_goal = (current_pos[0], robot_goal[1])

            move = await self.move_to_goal(temp_goal)

            # only exit the handler on hard outcomes; otherwise keep looping until 'CLEARED'
            if move in ("replanned", "FAILED"):
                return move

            await asyncio.sleep(0.01)

    async def _read_and_clear_cmd(self):
        """Read shared_cmd.json once and clear it. Return dict or None."""
        if not os.path.exists(self.CMD_FILE):
            return None
        try:
            with open(self.CMD_FILE, "r") as f:
                data = json.load(f)
            # best-effort clear so we don't reconsume the same command
            try:
                os.remove(self.CMD_FILE)
            except Exception:
                pass
            return data
        except Exception as e:
            print(f"⚠️ failed to read {self.CMD_FILE}: {e}")
            return None

    async def _await_llm_decision(self, robot_name: str, blocked_dir: str):
        """
        Wait for user/LLM action on non-robot obstacle.
        Valid actions:
          wait      -> keep waiting while periodically checking if clear
          continue  -> proceed with our automatic handler (detour)
          stop      -> abort path ("FAILED")
          go_home   -> request main to send a go_home; return "replanned"
          set_goal  -> LLM will write shared_goal.json; return "replanned"
        """
        print(f"⏸️ {robot_name} paused due to non-robot obstacle ({blocked_dir}). Waiting for LLM…")
        while True:
            # 1) if it cleared by itself, just continue
            dirs, robot_info = check_sensors_for_obstacle(0.0, 0.0, robot_name)
            if not any(d == blocked_dir for d in dirs):
                print("🟢 Obstacle cleared during wait.")
                return "CLEARED_AUTO"

            # 2) check for a command
            cmd = await self._read_and_clear_cmd()
            if cmd:
                ctype = cmd.get("type")
                target = cmd.get("robot_id")
                if target and target != robot_name:
                    # command for a different robot; ignore
                    pass
                else:
                    if ctype == "obstacle_action":
                        action = (cmd.get("action") or "").lower()
                        print(f"📥 LLM action received: {action}")
                        if action == "wait":
                            # just keep looping
                            pass
                        elif action == "continue":
                            return "CONTINUE"
                        elif action == "stop":
                            return "FAILED"
                        elif action == "go_home":
                            # main loop will react to 'go_home' or you can set a flag here
                            return "REPLAN"
                        elif action == "set_goal":
                            # LLM should have written shared_goal.json already
                            return "REPLAN"

                    # also accept legacy types from LLM.py for convenience
                    if ctype in ("stop", "go_home"):
                        return "FAILED" if ctype == "stop" else "REPLAN"

            await asyncio.sleep(0.2)

