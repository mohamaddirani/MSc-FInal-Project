# sim_app/robot_controller.py

class OmniRobotController:
    """
    Controller class for an omnidirectional robot in a simulation environment.
    Methods
    -------
    __init__():
        Initializes the controller state.
    async init_handles(sim, robot_name):
        Asynchronously initializes simulation object handles for the robot and its wheels.
        Sets the joint force for each wheel motor and marks the controller as initialized.
    async get_position():
        Asynchronously retrieves the current (x, y) position of the robot in the simulation.
    """
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
            await sim.setJointTargetForce(joint, 100)
        self.initialized = True
        print(f"🔧 {robot_name} controller initialized.")
        
    async def get_position(self):
        pos = await self.sim.getObjectPosition(self.robot, -1)
        return pos[:2]

    async def get_orientation(self):
        ori = await self.sim.getObjectOrientation(self.robot, -1)
        print(f"Orientation of {self.robot}: {ori}")
        return ori
