import math
import asyncio
import numpy as np
from sim_app.shared import latest_data

OBSTACLE_THRESHOLD = 0.5      # meters
SPEED = 0.3                   # m/s
WHEEL_RADIUS = 0.05           # meters
SAFE_STOP_ATTEMPTS = 10       # limit retries before emergency stop
SLOWDOWN_DISTANCE = 0.2       # start slowing down near the goal

class OmniRobotController:
    def __init__(self):
        self.initialized = False
        self.obstacle_attempts = 0

    async def get_yaw(self):
        euler = await self.sim.getObjectOrientation(self.robot, -1)
        return euler[2]  # yaw (Z-axis rotation)

    async def init_handles(self, sim):
        self.wheels = {
            "fl": await sim.getObject('/Omnirob0/FLwheel_motor'),
            "fr": await sim.getObject('/Omnirob0/FRwheel_motor'),
            "rl": await sim.getObject('/Omnirob0/RLwheel_motor'),
            "rr": await sim.getObject('/Omnirob0/RRwheel_motor'),
        }
        self.robot = await sim.getObject('/Omnirob0')
        self.sim = sim
        self.initialized = True
        print("🔧 Omniwheel controller initialized.")

    async def get_position(self):
        pos = await self.sim.getObjectPosition(self.robot, -1)
        return pos[:2]

    def distance(self, a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def obstacle_in_path(self, points, x, y, dx, dy):
        for pt in points:
            px, py = pt[0], pt[1]
            vx, vy = px - x, py - y
            dist = math.hypot(vx, vy)

            if dist < OBSTACLE_THRESHOLD:
                dot = (vx * dx + vy * dy) / (dist + 1e-6)
                angle = math.acos(max(-1, min(1, dot)))
                if angle < math.radians(25):
                    return True
        return False

    def compute_wheel_velocities(self, vx, vy):
        """
        Converts robot-local vx, vy velocities into individual wheel velocities.
        Wheel mounting verified using:
            - FL: forward-left
            - FR: forward-right
            - RL: rear-left
            - RR: rear-right

        Verified against the Lua test:
            forward  => FL:+v, FR:-v, RR:-v, RL:+v
            backward => FL:-v, FR:+v, RR:+v, RL:-v
            right    => FL:+v, FR:+v, RR:-v, RL:-v
            left     => FL:-v, FR:-v, RR:+v, RL:+v
        """

        fl =  vx + vy  # ⬆️ + ➡️
        fr = -vx + vy  # ⬆️ - ➡️
        rr = -vx - vy  # ⬇️ - ➡️
        rl =  vx - vy  # ⬇️ + ➡️

        # Normalize if needed
        max_val = max(abs(fl), abs(fr), abs(rl), abs(rr))
        if max_val > 1:
            fl /= max_val
            fr /= max_val
            rl /= max_val
            rr /= max_val

        scale = SPEED / WHEEL_RADIUS
        return fl * scale, fr * scale, rl * scale, rr * scale


    async def stop(self):
        print("🛑 Emergency Stop.")
        for handle in self.wheels.values():
            await self.sim.setJointTargetVelocity(handle, 0)

    async def move_to_goal(self, goal):
        current = await self.get_position()
        dx = goal[0] - current[0]
        dy = goal[1] - current[1]
        dist = math.hypot(dx, dy)

        print(f"📍 Distance to goal: {dist:.2f} meters")

        if dist < 0.05:
            print("🎯 Goal reached. Stopping.")
            await self.stop()
            return True

        # Get robot orientation
        theta = await self.get_yaw()
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        # Transform world delta into robot-local frame
        vx_world = 0
        vy_world = 0

        # Align Y first (forward/backward movement)
        if abs(dy) > 0.05:
            vy_world = SPEED if dy > 0 else -SPEED
            print("⬆️ Aligning Y axis")

        # Then align X (sideways movement)
        elif abs(dx) > 0.05:
            vx_world = SPEED if dx > 0 else -SPEED
            print("➡️ Moving along X axis")

        # Convert to robot-local velocities
        vx_local =  cos_t * vx_world + sin_t * vy_world
        vy_local = -sin_t * vx_world + cos_t * vy_world

        # Calculate wheel velocities
        w_fl, w_fr, w_rl, w_rr = self.compute_wheel_velocities(vx_local, vy_local)
        await self.sim.setJointTargetVelocity(self.wheels["fl"], w_fl)
        await self.sim.setJointTargetVelocity(self.wheels["fr"], w_fr)
        await self.sim.setJointTargetVelocity(self.wheels["rl"], w_rl)
        await self.sim.setJointTargetVelocity(self.wheels["rr"], w_rr)

        return False
