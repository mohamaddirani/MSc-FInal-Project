# sim_app/map_builder.py
"""
Map building utilities:
- Stamp latest sensor hits into the global occupancy grid.
- Inflate occupancy into a costmap (square neighborhood, original behavior).
- Provide a planning-view costmap without mutating globals.
"""

import numpy as np
import sim_app.shared as shared  # import the module, not names


# ============================================================================
# Helpers
# ============================================================================

def _center_indices(arr: np.ndarray) -> tuple[int, int]:
    """Return (cx, cy) indices of the array center."""
    return arr.shape[1] // 2, arr.shape[0] // 2


def _world_to_grid(x: float, y: float, grid: np.ndarray, res: float) -> tuple[int, int]:
    """Convert world (meters) → grid indices for the given grid and resolution."""
    cx, cy = _center_indices(grid)
    gx = int(round(cx + x / res))
    gy = int(round(cy + y / res))
    return gx, gy


def _stamp_hit(grid: np.ndarray, wx: float, wy: float, res: float) -> None:
    """Mark cell as occupied (1) if (wx, wy) falls inside the grid."""
    gx, gy = _world_to_grid(wx, wy, grid, res)
    if 0 <= gx < grid.shape[1] and 0 <= gy < grid.shape[0]:
        grid[gy, gx] = 1


def _inflate(binary_occ: np.ndarray, radius_m: float, res: float) -> np.ndarray:
    """
    Simple square-neighborhood inflation (keeps your original behavior).

    Args:
        binary_occ: uint8 occupancy (0/1).
        radius_m: inflation radius in meters.
        res: meters per cell.

    Returns:
        float32 costmap where inflated cells are 1.0, free are 0.0.
    """
    if radius_m <= 0:
        return binary_occ.astype(np.float32)

    k = max(1, int(round(radius_m / res)))
    H, W = binary_occ.shape
    out = np.zeros_like(binary_occ, dtype=np.uint8)

    for y in range(H):
        y0 = max(0, y - k)
        y1 = min(H, y + k + 1)
        row = out[y]
        for x in range(W):
            x0 = max(0, x - k)
            x1 = min(W, x + k + 1)
            if binary_occ[y0:y1, x0:x1].any():
                row[x] = 1

    return out.astype(np.float32)


# ============================================================================
# Mapping (unchanged behavior)
# ============================================================================

def update_memory_with_latest(robot_name: str) -> None:
    """
    Stamp all latest points for this robot into the CURRENT global occupancy.
    Respects shared.FREEZE_MAP (no-op if True).
    """
    if getattr(shared, "FREEZE_MAP", False):
        return

    rx, ry = shared.robot_positions.get(robot_name, (0.0, 0.0))
    for sig in (f"{robot_name}_S300_combined_data", f"{robot_name}_S3001_combined_data"):
        for (lx, ly, _lz, _dist) in shared.latest_data.get(sig, []):
            wx, wy = rx + lx, ry + ly
            _stamp_hit(shared.global_occupancy, wx, wy, shared.MAP_RESOLUTION)


def rebuild_costmap(inflation_radius_m: float = shared.INFLATION_RADIUS_M) -> None:
    """
    Rebuild the global costmap by inflating occupancy.
    Thickness is controlled by inflation_radius_m (meters).
    """
    if inflation_radius_m is None:
        inflation_radius_m = getattr(shared, "COST_INFLATION_RADIUS_M", 0.10)

    cm = _inflate(shared.global_occupancy, inflation_radius_m, shared.MAP_RESOLUTION)
    shared.global_costmap[:] = cm
    shared.maybe_autosave(every_n_updates=25)


# ============================================================================
# Optional: simple planning view (no clearing)
# ============================================================================

def get_planning_costmap(inflation_radius_m: float) -> np.ndarray:
    """
    Returns a *copy* of the current costmap built with the given inflation.
    Does NOT clear robot footprints; does NOT mutate the globals.
    """
    occ = shared.global_occupancy.astype(np.uint8, copy=True)
    return _inflate(occ, inflation_radius_m, shared.MAP_RESOLUTION)
