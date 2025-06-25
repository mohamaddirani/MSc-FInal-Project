import math
import asyncio
from sim_app.shared import latest_data
from sim_app.obstacle_awareness import is_path_clear

# Constants
OBSTACLE_THRESHOLD = 0.5       # meters
SPEED = 0.3                    # m/s
WHEEL_RADIUS = 0.05            # meters
SAFE_STOP_ATTEMPTS = 10
SLOWDOWN_DISTANCE = 0.2

class OmniRobotController:
    def __init__(self):
        self.initialized = False
        self.obstacle_attempts = 0
        self.phase = "x"  # Movement phase: 'x', then 'y', then 'done'

    async def get_yaw(self):
        euler = await self.sim.getObjectOrientation(self.robot, -1)
        return euler[2]

    async def init_handles(self, sim):
        self.sim = sim
        self.wheels = {
            "fl": await sim.getObject('/Omnirob0/FLwheel_motor'),
            "fr": await sim.getObject('/Omnirob0/FRwheel_motor'),
            "rl": await sim.getObject('/Omnirob0/RLwheel_motor'),
            "rr": await sim.getObject('/Omnirob0/RRwheel_motor'),
        }
        self.robot = await sim.getObject('/Omnirob0')

        # ✅ Set proper joint force for motion
        for joint in self.wheels.values():
            await sim.setJointForce(joint, 100)

        self.initialized = True
        print("🔧 Omniwheel controller initialized.")

    async def get_position(self):
        pos = await self.sim.getObjectPosition(self.robot, -1)
        return pos[:2]

    async def stop(self):
        print("🛑 Emergency Stop.")
        for handle in self.wheels.values():
            await self.sim.setJointTargetVelocity(handle, 0)

    async def move_to_goal(self, goal):
        current = await self.get_position()
        dx = goal[0] - current[0]
        dy = goal[1] - current[1]
        dist = math.hypot(dx, dy)

        print(f"📍 Current position: {current}")
        print(f"dx: {dx:.2f}, dy: {dy:.2f}")
        print(f"📍 Distance to goal: {dist:.2f} meters")

        if dist < 0.05:
            print("🎯 Goal reached. Stopping.")
            await self.stop()
            return True

        vx, vy = 0.0, 0.0

        if self.phase == "x":
            if abs(dx) < 0.01:
                print("✅ X axis aligned. Switching to Y axis.")
                self.phase = "y"
                return False

            direction = "right" if dx > 0 else "left"
            if not is_path_clear(direction):
                print(f"❌ Blocked on {direction} during X alignment. Switching to Y phase temporarily.")
                self.phase = "y"
                return False

            self.obstacle_attempts = 0
            vx = -SPEED if dx > 0 else SPEED
            print(f"➡️ Aligning X axis (moving {direction})")

        elif self.phase == "y":
            if abs(dy) < 0.05:
                print("✅ Y axis aligned.")
                # Switch back to X if not yet fully aligned
                if abs(dx) > 0.01:
                    print("↩️ Returning to X axis for final alignment.")
                    self.phase = "x"
                    return False
                self.phase = "done"
                await self.stop()
                return True

            direction = "back" if dy > 0 else "front"
            if not is_path_clear(direction):
                print(f"❌ Blocked on {direction} during Y alignment. Returning to X phase.")
                self.phase = "x"
                return False

            self.obstacle_attempts = 0
            vy = -SPEED if dy > 0 else SPEED
            print(f"⬆️ Aligning Y axis (moving {direction})")

        # Wheel velocities
        fl_vel = fr_vel = rl_vel = rr_vel = 0.0

        if vx != 0.0:
            fl_vel =  vx
            fr_vel =  vx
            rl_vel = -vx
            rr_vel = -vx
        elif vy != 0.0:
            fl_vel = vy
            fr_vel = -vy
            rl_vel = vy
            rr_vel = -vy

        scale = 1 / WHEEL_RADIUS
        print(f"⚙️ Wheel Speeds: FL={fl_vel:.2f}, FR={fr_vel:.2f}, RL={rl_vel:.2f}, RR={rr_vel:.2f}")

        await self.sim.setJointTargetVelocity(self.wheels["fl"], fl_vel * scale)
        await self.sim.setJointTargetVelocity(self.wheels["fr"], fr_vel * scale)
        await self.sim.setJointTargetVelocity(self.wheels["rl"], rl_vel * scale)
        await self.sim.setJointTargetVelocity(self.wheels["rr"], rr_vel * scale)

        return False

