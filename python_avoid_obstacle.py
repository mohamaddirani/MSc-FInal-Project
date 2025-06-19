from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import time

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.setStepping(False)
print("✅ Connected to CoppeliaSim via ZeroMQ Remote API")

# Get Omnirob0 handle
omnirob = sim.getObject('/Omnirob0')

# Get all objects under Omnirob0
children = sim.getObjectsInTree(omnirob, -1, 0)

# Identify laser script path
laser_script_alias = None
for h in children:
    alias = sim.getObjectAlias(h, 1)
    if alias.endswith("/SickS300/script"):
        laser_script_alias = alias
        break

if not laser_script_alias:
    raise RuntimeError("❌ Could not find SickS300/script")

print(f"📡 Found laser script: {laser_script_alias}")

# Get motor handles
left_motor = sim.getObject('/Omnirob0/RLwheel_motor')
right_motor = sim.getObject('/Omnirob0/RRwheel_motor')

# Control loop
start_time = time.time()
duration = 20  # seconds

while time.time() - start_time < duration:
    try:
        raw_data = sim.callScriptFunction(f"getLaserData@{laser_script_alias}", sim.scripttype_childscript)

        if raw_data:
            raw_data = raw_data.encode("latin1")
            unpacked = sim.unpackFloatTable(raw_data)
            points = np.array(unpacked).reshape(-1, 3)
            distances = np.linalg.norm(points, axis=1)
            min_distance = np.min(distances)

            vl, vr = (-2.0, 2.0) if min_distance < 0.6 else (2.0, 2.0)
            print(f"📏 Distance: {min_distance:.2f} → L={vl}, R={vr}")
        else:
            print("⚠️ No laser data received.")
            vl, vr = 0.0, 0.0

        sim.setJointTargetVelocity(left_motor, vl)
        sim.setJointTargetVelocity(right_motor, vr)

    except Exception as e:
        print(f"❌ Loop error: {e}")
        break

    time.sleep(0.2)

# Stop robot
sim.setJointTargetVelocity(left_motor, 0.0)
sim.setJointTargetVelocity(right_motor, 0.0)
print("⏹️ Stopped robot.")
