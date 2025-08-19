"""
Global shared state for the sim.
- Persistent occupancy/cost maps that grow as needed
- Latest sensor packets (already in robot-local laser_frame)
- Robot poses/goals/status
"""
from collections import defaultdict
import os, math, numpy as np

# --- map config ---
MAP_RESOLUTION = 0.2                            # meters / cell
GRID_SIZE = 225                                 # keep odd size so center is an integer cell
MAP_SIZE_M = GRID_SIZE * MAP_RESOLUTION
INFLATION_RADIUS_M = 0.10
FREEZE_MAP = True    # True = use the saved map only; no live updates
# --- persistent map memory (in RAM) ---
global_occupancy = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)    # 0=free, 1=hit
global_costmap   = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)  # 0.0=free, 1.0=inflated

# file on disk
MAP_FILE = os.path.join(os.path.dirname(__file__), "map_memory.npz")

def save_map(path: str = MAP_FILE):
    """Persist occupancy + costmap to disk."""
    np.savez_compressed(path,
                        occupancy=global_occupancy,
                        costmap=global_costmap,
                        res=np.float32(MAP_RESOLUTION))

def load_map(path: str = MAP_FILE):
    """Load map from disk if present."""
    global global_occupancy, global_costmap, GRID_SIZE, MAP_SIZE_M
    if os.path.exists(path):
        data = np.load(path)
        global_occupancy = data["occupancy"].astype(np.uint8)
        global_costmap   = data["costmap"].astype(np.float32)
        GRID_SIZE = int(global_occupancy.shape[0])
        MAP_SIZE_M = GRID_SIZE * MAP_RESOLUTION

def clear_map():
    """Reset in-RAM map (does not delete the .npz file)."""
    global global_occupancy, global_costmap
    global_occupancy[:] = 0
    global_costmap[:]   = 0.0

# autosave helper (call occasionally)
def maybe_autosave(every_n_updates: int = 50):
    global _save_counter
    try:
        _save_counter += 1
    except NameError:
        _save_counter = 1
    if _save_counter % every_n_updates == 0:
        save_map()

def ensure_map_covers(xs, ys, margin_m: float = 1.0):
    """
    Grow global_occupancy/global_costmap symmetrically if any (x,y) lies
    outside the current map (plus a margin).
    """
    global global_occupancy, global_costmap, GRID_SIZE, MAP_SIZE_M
    half_w = (GRID_SIZE // 2) * MAP_RESOLUTION
    need = max(max(abs(x) for x in xs), max(abs(y) for y in ys)) + margin_m
    if need <= half_w:
        return
    new_half_cells = math.ceil(need / MAP_RESOLUTION)
    new_size = int(new_half_cells * 2 + 1)
    pad = (new_size - GRID_SIZE) // 2
    if pad <= 0:
        return
    global_occupancy = np.pad(global_occupancy, ((pad, pad), (pad, pad)), mode='constant')
    global_costmap   = np.pad(global_costmap,   ((pad, pad), (pad, pad)), mode='constant')
    GRID_SIZE = new_size
    MAP_SIZE_M = GRID_SIZE * MAP_RESOLUTION

# ---------- dynamic runtime state ----------
latest_data     = defaultdict(list)   # signal_name -> [(x,y,z,dist), ...] in robot-local (laser_frame)
all_sensor_data = defaultdict(list)

latest_astar_path: list = []
executed_path:     list = []
planned_path:      list = []

robot_start_positions = {
    "Rob0": (0.0, 0.0),
    "Rob1": (0.0, 0.0),
    "Rob2": (0.0, 0.0),
}
# Robot world positions (updated by PathExecutor each step)
robot_positions = {
    "Rob0": (0.0, 0.0),
    "Rob1": (0.0, 0.0),
    "Rob2": (0.0, 0.0),
}
robot_orientation = {
    "Rob0": (0.0, 0.0, 0.0),
    "Rob1": (0.0, 0.0, 0.0),
    "Rob2": (0.0, 0.0, 0.0),
}

# Goals per robot
robot_goal = {
    "Rob0": None,
    "Rob1": None,
    "Rob2": None,
}

# Status per robot
robot_status = {
    "Rob0": "idle",
    "Rob1": "idle",
    "Rob2": "idle",
}

message_log: list = []
