# sim_app/sensor_fetch.py
"""
Utilities to fetch and decode sensor packets from CoppeliaSim string signals.

- Signals are ASCII Base64 strings containing little-endian float32 sequences.
- Supports payloads of (x, y, z, dist) or (x, y, z) per point.
- Populates `latest_data` and appends to `all_sensor_data`.
"""

import base64
import struct
from math import sqrt

from sim_app.shared import all_sensor_data, latest_data


async def fetch_sensor_data(sim, signal_name: str) -> int:
    """
    Fetch and decode a sensor packet for `signal_name`.

    Args:
        sim: CoppeliaSim remote API object.
        signal_name (str): The name of the string signal to read.

    Returns:
        int: Number of decoded points.
    """
    # ASCII Base64 text from the sim
    b64 = await sim.getStringSignal(signal_name)
    if not b64:
        latest_data[signal_name] = []
        return 0

    if isinstance(b64, str):
        b64 = b64.encode("ascii")

    # Back to raw binary bytes
    raw = base64.b64decode(b64)

    n = len(raw) // 4
    if n == 0:
        latest_data[signal_name] = []
        return 0

    # Little-endian float32
    floats = struct.unpack("<" + "f" * n, raw)

    pts = []
    if n % 4 == 0:
        # Format: (x, y, z, dist)
        for i in range(0, len(floats), 4):
            x, y, z, dist = floats[i : i + 4]
            pts.append((x, y, z, dist))
    elif n % 3 == 0:
        # Format: (x, y, z) — derive dist
        for i in range(0, len(floats), 3):
            x, y, z = floats[i : i + 3]
            pts.append((x, y, z, sqrt(x * x + y * y)))
    else:
        latest_data[signal_name] = []
        return 0

    latest_data[signal_name] = pts
    all_sensor_data[signal_name].extend(pts)
    return len(pts)
