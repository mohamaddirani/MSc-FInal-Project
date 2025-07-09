# sim_app/robot_motion.py

SPEED = 0.1
WHEEL_RADIUS = 0.05

class RobotMotion:
    def __init__(self, sim, wheels):
        self.sim = sim
        self.wheels = wheels

    async def stop(self):
        print("🛑 Stopping robot.")
        for joint in self.wheels.values():
            await self.sim.setJointTargetVelocity(joint, 0)

    async def set_velocity(self, vx, vy):#, dx, dy):
        fl_vel = fr_vel = rl_vel = rr_vel = 0.0
        # if abs(vy) > abs(vx):  # Dominant X motion
        #     fl_vel = -vx
        #     fr_vel = -vx
        #     rr_vel = vx
        #     rl_vel = vx
        # elif abs(vx) > abs(vy):  # Dominant Y motion
        #     fl_vel = -vy
        #     rl_vel = -vy
        #     fr_vel = vy
        #     rr_vel = vy
        # else:
        #     print("No significant movement detected, stopping.")
        #     fl_vel = fr_vel = rl_vel = rr_vel = 0
        #dx < 0 -> right
        #dy < 0 -> back
        # if abs(vy) < 0.05:
        #     if dx < 0: #Right
        #         fl_vel = vx + vy
        #         fr_vel = -vx - vy
        #         rl_vel = -vx - vy
        #         rr_vel = vx + vy
        #     elif dx > 0: #Left
        #         fl_vel = -vx - vy
        #         fr_vel = vx + vy
        #         rl_vel = vx + vy
        #         rr_vel = -vx - vy
        # elif abs(vx) < 0.05:
        #     if dy > 0: #Front
        #         fl_vel = vx + vy
        #         fr_vel = vx + vy
        #         rl_vel = vx + vy
        #         rr_vel = vx + vy
        #     elif dy < 0: #Back
        #         fl_vel = -vx - vy
        #         fr_vel = -vx - vy
        #         rl_vel = -vx - vy
        #         rr_vel = -vx - vy
        # elif dx < 0 and dy > 0: #front right
        #     fl_vel =  vx + vy
        #     fr_vel = 0
        #     rl_vel = 0
        #     rr_vel = vx + vy
        # elif dx < 0 and dy < 0: #Back Right
        #     fl_vel = 0
        #     fr_vel = -vx - vy
        #     rl_vel = -vx - vy
        #     rr_vel = 0
        # elif dx > 0 and dy > 0: #Front Left
        #     fl_vel = 0
        #     fr_vel = vx + vy
        #     rl_vel = vx + vy
        #     rr_vel = 0
        # elif dx > 0 and dy < 0: #Back Left
        #     fl_vel = -vx - vy
        #     fr_vel = 0
        #     rl_vel = 0
        #     rr_vel = -vx - vy
        # vel_x = vel_y = 0
         
        # if abs(vx) > abs(vy):
        #     vel_y = vel_x = vx

        # elif abs(vx) < abs(vy):
        #     vel_x = vel_y = vy
        # else:
        #     vel_x = vx
        #     vel_y = vy
        # Allow diagonal motion by combining vx and vy properly
        
        #vy = -vy# Reverse vy because dy > 0 means back
        fl_vel = -vy - vx
        fr_vel = vy - vx
        rl_vel = -vy + vx
        rr_vel = vy + vx







        # print(f"vel_x {vel_x}  vel_y {vel_y}")
        print(f"vx {vx}, vy {vy}, fl_vel {fl_vel}, fr_vel {fr_vel}, rl_vel{rl_vel}, rr_vel {rr_vel}")
        scale = 1 / WHEEL_RADIUS
        
        await self.sim.setJointTargetVelocity(self.wheels["fl"], fl_vel * scale)
        await self.sim.setJointTargetVelocity(self.wheels["fr"], fr_vel * scale)
        await self.sim.setJointTargetVelocity(self.wheels["rl"], rl_vel * scale)
        await self.sim.setJointTargetVelocity(self.wheels["rr"], rr_vel * scale)
