# sim_app/robot_controller.py

class OmniRobotController:
    def __init__(self):
        self.initialized = False

    async def init_handles(self, sim, robot_name):
        self.sim = sim
        self.wheels = {
            "fl": await sim.getObject(f'/{robot_name}/FLwheel_motor'),
            "fr": await sim.getObject(f'/{robot_name}/FRwheel_motor'),
            "rl": await sim.getObject(f'/{robot_name}/RLwheel_motor'),
            "rr": await sim.getObject(f'/{robot_name}/RRwheel_motor'),
        }
        self.robot = await sim.getObject(f'/{robot_name}')
        for joint in self.wheels.values():
            await sim.setJointForce(joint, 100)
        self.initialized = True
        print(f"🔧 {robot_name} controller initialized.")
        
    async def get_position(self):
        pos = await self.sim.getObjectPosition(self.robot, -1)
        return pos[:2]
