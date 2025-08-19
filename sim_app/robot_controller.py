# sim_app/robot_controller.py
class OmniRobotController:
    """
    Holds the robot handle after init_handles and exposes no-arg getters.
    """
    def __init__(self, sim=None):
        self.initialized = False
        self.sim = sim
        self.robot = None
        self.wheels = {}

    async def init_handles(self, sim, robot_name):
        self.sim = sim
        self.wheels = {
            "fl": await sim.getObject(f'/{robot_name}/FLwheel_motor'),
            "fr": await sim.getObject(f'/{robot_name}/FRwheel_motor'),
            "rl": await sim.getObject(f'/{robot_name}/RLwheel_motor'),
            "rr": await sim.getObject(f'/{robot_name}/RRwheel_motor'),
        }
        self.robot = await sim.getObject(f'/{robot_name}')  # <-- numeric handle
        for j in self.wheels.values():
            await sim.setJointTargetForce(j, 100)
        self.initialized = True
        print(f"🔧 {robot_name} controller initialized.")

    async def get_position(self):
        pos = await self.sim.getObjectPosition(self.robot, -1)
        return pos[:2]

    async def get_orientation(self):
        ori = await self.sim.getObjectOrientation(self.robot, -1)
        return ori
