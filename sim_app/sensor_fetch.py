# sim_app/sensor_fetch.py
import base64, struct
from math import sqrt
from sim_app.shared import latest_data, all_sensor_data

async def fetch_sensor_data(sim, signal_name: str) -> int:
    b64 = await sim.getStringSignal(signal_name)   # ASCII Base64 text
    if not b64:
        latest_data[signal_name] = []
        return 0
    if isinstance(b64, str):
        b64 = b64.encode('ascii')
    raw = base64.b64decode(b64)                    # back to binary bytes

    n = len(raw) // 4
    if n == 0:
        latest_data[signal_name] = []
        return 0

    floats = struct.unpack('<' + 'f'*n, raw)       # little-endian float32

    pts = []
    if n % 4 == 0:
        for i in range(0, len(floats), 4):
            x, y, z, dist = floats[i:i+4]
            pts.append((x, y, z, dist))
    elif n % 3 == 0:
        for i in range(0, len(floats), 3):
            x, y, z = floats[i:i+3]
            pts.append((x, y, z, sqrt(x*x + y*y)))
    else:
        latest_data[signal_name] = []
        return 0

    latest_data[signal_name] = pts
    all_sensor_data[signal_name].extend(pts)
    return len(pts)
