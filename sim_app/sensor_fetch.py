# sim_app/sensor_fetch.py
import base64
import cbor2
from sim_app.shared import latest_data

async def fetch_sensor_data(sim, signal_name):
    raw = await sim.getStringSignal(signal_name)
    if raw:
        try:
            decoded = base64.b64decode(raw)
            points = cbor2.loads(decoded)
            if isinstance(points, list):
                latest_data[signal_name] = points
            else:
                latest_data[signal_name] = []

            points = latest_data.get(signal_name, [])
            if points:
                closest_dist = float('inf')
                closest_point = None
                for pt in points:
                    if len(pt) >= 4:
                        x, y, z, dist = pt[:4]
                        if dist < closest_dist:
                            closest_dist = dist
                            closest_point = (x, y, dist)
                    else:
                        print(f"⚠️ Unexpected point format in {signal_name}: {pt}")
                return closest_point if closest_point is not None else None
            else:
                return None

        except Exception as e:
            print(f"❌ Failed to decode sensor data for {signal_name}: {e}")
            latest_data[signal_name] = []
            return None
    else:
        latest_data[signal_name] = []
        return None
