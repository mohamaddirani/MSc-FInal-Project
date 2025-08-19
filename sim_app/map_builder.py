# sim_app/map_builder.py
import numpy as np
import sim_app.shared as shared
import math

def _center_indices(arr: np.ndarray):
    return arr.shape[1] // 2, arr.shape[0] // 2

def _world_to_grid(x: float, y: float, grid: np.ndarray, res: float):
    cx, cy = _center_indices(grid)
    gx = int(round(cx + x / res))
    gy = int(round(cy + y / res))
    return gx, gy

def _stamp_hit(grid: np.ndarray, wx: float, wy: float, res: float):
    gx, gy = _world_to_grid(wx, wy, grid, res)
    if 0 <= gx < grid.shape[1] and 0 <= gy < grid.shape[0]:
        grid[gy, gx] = 1

def _inflate(binary_occ: np.ndarray, radius_m: float, res: float) -> np.ndarray:
    """
    Circular (disk) dilation instead of square (reduces false 'walls').
    Returns float32 map with 0.0/1.0.
    """
    if radius_m <= 0:
        return binary_occ.astype(np.float32)

    k = max(1, int(round(radius_m / res)))
    H, W = binary_occ.shape
    out = np.zeros_like(binary_occ, dtype=np.uint8)

    # precompute disk mask
    yy, xx = np.ogrid[-k:k+1, -k:k+1]
    disk = (xx*xx + yy*yy) <= (k*k)

    for y in range(H):
        y0 = max(0, y - k); y1 = min(H, y + k + 1)
        for x in range(W):
            x0 = max(0, x - k); x1 = min(W, x + k + 1)
            sub = binary_occ[y0:y1, x0:x1]
            m   = disk[(y0 - (y - k)):(y1 - (y - k)), (x0 - (x - k)):(x1 - (x - k))]
            if (sub[m]).any():
                out[y, x] = 1

    return out.astype(np.float32)

def update_memory_with_latest(robot_name: str):
    """
    ✅ FIX: stamp points with the robot's **current yaw** so local (laser_frame)
    points are rotated into world before stamping.
    Make sure you keep shared.robot_orientation[robot_name] updated each step.
    """
    rx, ry = shared.robot_positions.get(robot_name, (0.0, 0.0))
    # yaw (rad) from shared (update this every control tick)
    yaw = 0.0
    if robot_name in shared.robot_orientation:
        # assuming (roll, pitch, yaw)
        yaw = float(shared.robot_orientation[robot_name][2])
    c, s = math.cos(yaw), math.sin(yaw)

    for sig in (f"{robot_name}_S300_combined_data",
                f"{robot_name}_S3001_combined_data"):
        for (lx, ly, _lz, _dist) in shared.latest_data.get(sig, []):
            # rotate local → world
            wx = rx + (c * lx - s * ly)
            wy = ry + (s * lx + c * ly)
            _stamp_hit(shared.global_occupancy, wx, wy, shared.MAP_RESOLUTION)

def rebuild_costmap(inflation_radius_m: float = 0.30):
    """
    Slightly smaller default inflation than 0.45 m to avoid over-blocking.
    """
    cm = _inflate(shared.global_occupancy, inflation_radius_m, shared.MAP_RESOLUTION)
    shared.global_costmap[:] = cm
