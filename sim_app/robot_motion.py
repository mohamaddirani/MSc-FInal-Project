# sim_app/robot_motion.py
"""
Low-level wheel commands for the omni robot.

Notes:
- Uses fixed wheel-speed patterns (no magnitude scaling) to ensure
  axis-aligned and diagonal motions match your original Lua-like behavior.
- Coordinate convention (robot/body frame):
    * "Horizontal"  → left/right movement (affects body +y)
    * "Vertical"    → forward/back movement (affects body +x)
    * "Diagonal"    → uses fixed patterns for the four diagonals
"""

import math

WHEEL_RADIUS = 0.05


class RobotMotion:
    def __init__(self, sim, wheels):
        self.sim = sim
        self.wheels = wheels

    async def stop(self):
        for j in self.wheels.values():
            await self.sim.setJointTargetVelocity(j, 0.0)

    async def set_velocity(self, dx, dy, move):
        """
        Set wheel target velocities based on desired motion classification.

        Args:
            dx (float): Desired delta along robot X (left/right chooser).
            dy (float): Desired delta along robot Y (front/back chooser).
            move (str): One of {"Horizontal", "Vertical", "Diagonal"}.

        Returns:
            tuple[float, float] | None: (vx, vy) command summary for logging,
            or None if `move` does not match any supported pattern.
        """
        v = 100 * math.pi / 180  # rad/s per wheel, Lua-like constant

        def sgn(a):
            return -1 if a < 0 else (1 if a > 0 else 0)

        if move == "Horizontal":
            s = sgn(dx)
            # left/right (body +y) mapping (no magnitude scaling!)
            await self.sim.setJointTargetVelocity(self.wheels["fl"], -s * v)
            await self.sim.setJointTargetVelocity(self.wheels["fr"], -s * v)
            await self.sim.setJointTargetVelocity(self.wheels["rr"],  s * v)
            await self.sim.setJointTargetVelocity(self.wheels["rl"],  s * v)
            return s * v, 0.0

        elif move == "Vertical":
            s = sgn(dy)
            # forward/back (body +x) mapping (no magnitude scaling!)
            await self.sim.setJointTargetVelocity(self.wheels["fl"], -s * v)
            await self.sim.setJointTargetVelocity(self.wheels["fr"],  s * v)
            await self.sim.setJointTargetVelocity(self.wheels["rr"],  s * v)
            await self.sim.setJointTargetVelocity(self.wheels["rl"], -s * v)
            return 0.0, s * v

        elif move == "Diagonal":
            # Use fixed patterns; do NOT multiply by |dx| or |dy|
            if dx > 0 and dy > 0:      # back-left
                await self.sim.setJointTargetVelocity(self.wheels["fl"], -2 * v)
                await self.sim.setJointTargetVelocity(self.wheels["fr"],  0)
                await self.sim.setJointTargetVelocity(self.wheels["rr"],  2 * v)
                await self.sim.setJointTargetVelocity(self.wheels["rl"],  0)
                return v,  v

            elif dx > 0 and dy < 0:    # front-left
                await self.sim.setJointTargetVelocity(self.wheels["fl"],  0)
                await self.sim.setJointTargetVelocity(self.wheels["fr"], -2 * v)
                await self.sim.setJointTargetVelocity(self.wheels["rr"],  0)
                await self.sim.setJointTargetVelocity(self.wheels["rl"],  2 * v)
                return v, -v

            elif dx < 0 and dy > 0:    # back-right
                await self.sim.setJointTargetVelocity(self.wheels["fl"],  0)
                await self.sim.setJointTargetVelocity(self.wheels["fr"],  2 * v)
                await self.sim.setJointTargetVelocity(self.wheels["rr"],  0)
                await self.sim.setJointTargetVelocity(self.wheels["rl"], -2 * v)
                return -v,  v

            elif dx < 0 and dy < 0:    # front-right
                await self.sim.setJointTargetVelocity(self.wheels["fl"],  2 * v)
                await self.sim.setJointTargetVelocity(self.wheels["fr"],  0)
                await self.sim.setJointTargetVelocity(self.wheels["rr"], -2 * v)
                await self.sim.setJointTargetVelocity(self.wheels["rl"],  0)
                return -v, -v
