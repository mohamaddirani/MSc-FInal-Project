import numpy as np
import sim_app.shared as shared  # <— import the module, not names

# --- helpers ---------------------------------------------------------------

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
    """Simple square neighborhood inflation (keeps your original behavior)."""
    if radius_m <= 0:
        return binary_occ.astype(np.float32)
    k = max(1, int(round(radius_m / res)))
    H, W = binary_occ.shape
    out = np.zeros_like(binary_occ, dtype=np.uint8)
    for y in range(H):
        y0 = max(0, y - k); y1 = min(H, y + k + 1)
        row = out[y]
        for x in range(W):
            x0 = max(0, x - k); x1 = min(W, x + k + 1)
            if binary_occ[y0:y1, x0:x1].any():
                row[x] = 1
    return out.astype(np.float32)

# --- mapping (unchanged behavior) ------------------------------------------

def update_memory_with_latest(robot_name: str):
    """Stamp all latest points for this robot into the CURRENT global occupancy."""
    if getattr(shared, "FREEZE_MAP", False):
        return
    rx, ry = shared.robot_positions.get(robot_name, (0.0, 0.0))
    for sig in (f"{robot_name}_S300_combined_data",
                f"{robot_name}_S3001_combined_data"):
        for (lx, ly, _lz, _dist) in shared.latest_data.get(sig, []):
            wx, wy = rx + lx, ry + ly
            _stamp_hit(shared.global_occupancy, wx, wy, shared.MAP_RESOLUTION)

def rebuild_costmap(inflation_radius_m: float = shared.INFLATION_RADIUS_M):
    """
    Rebuild the global costmap by inflating occupancy.
    Thickness is controlled by inflation_radius_m (meters).
    """
    if inflation_radius_m is None:
        inflation_radius_m = getattr(shared, "COST_INFLATION_RADIUS_M", 0.10)

    cm = _inflate(shared.global_occupancy, inflation_radius_m, shared.MAP_RESOLUTION)
    shared.global_costmap[:] = cm


# --- optional: simple planning view (no clearing) --------------------------
def get_planning_costmap(inflation_radius_m: float) -> np.ndarray:
    """
    Returns a *copy* of the current costmap built with the given inflation.
    Does NOT clear robot footprints; does NOT mutate the globals.
    """
    occ = shared.global_occupancy.astype(np.uint8, copy=True)
    return _inflate(occ, inflation_radius_m, shared.MAP_RESOLUTION)

