# sim_app/sensor_fetch.py
import base64
import cbor2
from sim_app.shared import latest_data

async def fetch_sensor_data(sim, signal_name):
    raw = await sim.getStringSignal(signal_name)
    if raw:
        decoded = base64.b64decode(raw)
        points = cbor2.loads(decoded)
        latest_data[signal_name] = points
        print(f"📡 [{signal_name}] {len(points)} points")
        for pt in points[:5]:
            x, y, z, dist = pt
            print(f"   -> x={x:.2f}, y={y:.2f}, z={z:.2f}, d={dist:.2f}")
    else:
        print(f"⏳ [{signal_name}] No data yet...")
        latest_data[signal_name] = []
