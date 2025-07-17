# sim_app/robot_motion.py

SPEED = 0.1
WHEEL_RADIUS = 0.05

class RobotMotion:
    """
    RobotMotion class provides methods to control the motion of a robot with four wheels in a simulation environment.
    Attributes:
        sim: The simulation interface object used to control the robot's joints.
        wheels: A dictionary mapping wheel names ("fl", "fr", "rl", "rr") to their respective joint handles.
    Methods:
        stop():
            Asynchronously stops all wheels of the robot by setting their target velocities to zero.
        set_velocity(vx, vy):
            Asynchronously sets the velocity of each wheel to achieve the desired linear velocities along the x (vx) and y (vy) axes.
            The velocities are calculated based on a mecanum wheel configuration.
            Args:
                vx (float): Desired velocity along the x-axis.
                vy (float): Desired velocity along the y-axis.
    """
    def __init__(self, sim, wheels):
        self.sim = sim
        self.wheels = wheels

    async def stop(self):
        for joint in self.wheels.values():
            await self.sim.setJointTargetVelocity(joint, 0)

    async def set_velocity(self, vx, vy):#, dx, dy):
        fl_vel = fr_vel = rl_vel = rr_vel = 0.0

        fl_vel = -vy - vx
        fr_vel = vy - vx
        rl_vel = -vy + vx
        rr_vel = vy + vx
        #print(f"vx {vx}, vy {vy}, fl_vel {fl_vel}, fr_vel {fr_vel}, rl_vel{rl_vel}, rr_vel {rr_vel}")
        scale = 1 / WHEEL_RADIUS
        
        await self.sim.setJointTargetVelocity(self.wheels["fl"], fl_vel * scale)
        await self.sim.setJointTargetVelocity(self.wheels["fr"], fr_vel * scale)
        await self.sim.setJointTargetVelocity(self.wheels["rl"], rl_vel * scale)
        await self.sim.setJointTargetVelocity(self.wheels["rr"], rr_vel * scale)
