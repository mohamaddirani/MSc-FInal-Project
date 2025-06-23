import sys
import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Constants
WHEEL_RADIUS = 0.05   # meters
BACKWARD_SPEED = -0.2 # m/s (negative for backward)
TEST_DURATION = 5     # seconds

def main():
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    print("✅ Connected to CoppeliaSim")

    sim.startSimulation()
    time.sleep(1)

    # Get wheel handles
    wheels = {
        'fl': sim.getObject('/Omnirob0/FLwheel_motor'),
        'fr': sim.getObject('/Omnirob0/FRwheel_motor'),
        'rl': sim.getObject('/Omnirob0/RLwheel_motor'),
        'rr': sim.getObject('/Omnirob0/RRwheel_motor'),
    }

    robot = sim.getObject('/Omnirob0')
    yaw = sim.getObjectOrientation(robot, -1)[2]
    print(f"🧭 Robot Yaw: {math.degrees(yaw):.2f}°")

    # Compute wheel velocities
    vx = BACKWARD_SPEED
    vy = 0
    scale = 1 / WHEEL_RADIUS
    wheel_velocities = {
        'fl': ( vx - vy) * scale,
        'fr': (-vx - vy) * scale,
        'rl': (-vx + vy) * scale,
        'rr': ( vx + vy) * scale,
    }

    for name, handle in wheels.items():
        sim.setJointTargetVelocity(handle, wheel_velocities[name])
    print("🚗 Moving backward...")

    time.sleep(TEST_DURATION)

    for handle in wheels.values():
        sim.setJointTargetVelocity(handle, 0)
    sim.stopSimulation()
    print("🛑 Robot stopped and simulation ended.")

if __name__ == "__main__":
    main()
