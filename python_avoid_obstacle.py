from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import time

PI = np.pi
robot_id = 0

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.setStepping(False)
print("✅ Connected to CoppeliaSim via ZeroMQ Remote API")

# Get wheel handles
left_motor = sim.getObject(f'/Omnirob[{robot_id}]/RLwheel_motor')
right_motor = sim.getObject(f'/Omnirob[{robot_id}]/RRwheel_motor')
print(f"✅ Found motors: Left={left_motor}, Right={right_motor}")

start_time = time.time()
duration = 60  # seconds

while (time.time() - start_time) < duration:
    try:
        sim.callScriptFunction("getLaserData@Omnirob[0]/SickS300_0_Script", sim.scripttype_customizationscript)

        if raw_data:
            if isinstance(raw_data, str):
                raw_data = raw_data.encode("latin1")

            unpacked = sim.unpackFloatTable(raw_data)
            points = np.array(unpacked).reshape(-1, 3)
            distances = np.linalg.norm(points, axis=1)
            min_distance = np.min(distances) if len(distances) > 0 else 1.0

            vl, vr = (-2.0, 2.0) if min_distance < 0.6 else (2.0, 2.0)
            print(f"📡 Obstacle @ {min_distance:.2f} → Velocities: L={vl}, R={vr}")
        else:
            print("🔴 No measured data. Stopping.")
            vl, vr = 0.0, 0.0

        sim.setJointTargetVelocity(left_motor, vl)
        sim.setJointTargetVelocity(right_motor, vr)

    except Exception as e:
        print(f"❌ Failed during loop: {e}")
        break

    time.sleep(0.2)

# Stop robot
try:
    sim.setJointTargetVelocity(left_motor, 0.0)
    sim.setJointTargetVelocity(right_motor, 0.0)
except Exception as e:
    print(f"⚠️ Could not stop motors: {e}")

print("⏹️ Stopped robot.")
