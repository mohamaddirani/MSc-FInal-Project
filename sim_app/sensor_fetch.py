# sim_app/sensor_fetch.py
import base64
import cbor2
from sim_app.shared import latest_data
from sim_app.shared import all_sensor_data

async def fetch_sensor_data(sim, signal_name):
    """
    Fetches and decodes sensor data from a simulation signal.
    This asynchronous function retrieves a base64-encoded, CBOR-serialized list of sensor data points from the specified signal in the simulation. It decodes and deserializes the data, updates global dictionaries with the latest and accumulated sensor data, and finds the point with the closest distance value.
    Args:
        sim: The simulation object providing the `getStringSignal` coroutine method.
        signal_name (str): The name of the signal to fetch sensor data from.
    Returns:
        tuple or None: A tuple (x, y, dist) representing the coordinates and distance of the closest point, or None if no valid data is found.
    Side Effects:
        Updates the global `latest_data` and `all_sensor_data` dictionaries with the latest and accumulated sensor data for the given signal.
    Raises:
        None. Exceptions are caught and logged; function returns None on failure.
    """
    raw = await sim.getStringSignal(signal_name)
    if raw:
        try:
            decoded = base64.b64decode(raw)
            points = cbor2.loads(decoded)
            if isinstance(points, list):
                latest_data[signal_name] = points
                all_sensor_data[signal_name].extend(points)

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
