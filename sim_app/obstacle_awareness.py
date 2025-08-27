# sim_app/obstacle_awareness.py
"""
Obstacle awareness utilities.

- Filters raw sensor hits into 8 directions (left/right/front/back + diagonals).
- Ignores static obstacles listed in occupied_grids.txt.
- Detects nearby robots and reports the nearest blocking robot + sector.
- Exposes:
    * check_sensors_for_obstacle(dx, dy, robot_name)
    * is_path_clear(direction, robot_name)
"""

import math
import os

import sim_app.shared as shared
from sim_app.astar_env import meters_to_grid


# ============================================================================
# Config
# ============================================================================

THRESHOLD_M = 0.75  # max range to consider "blocking"
DIAG_RATIO = 1      # dominance ratio to prefer cardinal over diagonal

# Candidate file locations
OCCUPIED_FILE_CANDIDATES = [
    os.path.join(os.path.dirname(__file__), "occupied_grids.txt"),
    "/mnt/data/occupied_grids.txt",
]


# ============================================================================
# File-backed occupied grid helpers
# ============================================================================

def _load_occupied_grids(paths=OCCUPIED_FILE_CANDIDATES):
    """Load occupied (blocked) grid cells from text files as (gx, gy) tuples."""
    occ = set()
    for path in paths:
        if not os.path.exists(path):
            continue
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line[0] != "(" or "," not in line:
                    continue
                try:
                    gx, gy = line.strip("()").split(",")
                    occ.add((int(gx), int(gy)))
                except Exception:
                    pass
        if occ:
            print(f"✅ loaded {len(occ)} occupied cells from: {path}")
            break
    if not occ:
        print("⚠️ no occupied_grids.txt found; every obstacle will be considered.")
    return occ


_OCCUPIED_CELLS = _load_occupied_grids()


def _obstacle_in_occupied_file(px: float, py: float) -> bool:
    """Return True if world (px, py) maps to a grid present in occupied_grids.txt."""
    g = shared.global_occupancy
    res = shared.MAP_RESOLUTION
    gx, gy = meters_to_grid(px, py, g, res)
    return (gx, gy) in _OCCUPIED_CELLS


# ============================================================================
# Sensor iteration & classification
# ============================================================================

def _iter_points(robot_name):
    """Iterate latest fused sensor points for this robot."""
    for sig in (f"{robot_name}_S300_combined_data", f"{robot_name}_S3001_combined_data"):
        for p in shared.latest_data.get(sig, []):
            yield p


def _direction8_of_point(p):
    """
    Classify a laser point (robot frame: +x=left, -x=right, +y=back, -y=front)
    into: left/right/front/back/front-left/front-right/back-left/back-right
    or return None if out of range.
    """
    x, y, *_rest = p
    dist = _rest[1] if len(_rest) >= 2 else math.hypot(x, y)
    if dist is None or dist >= THRESHOLD_M:
        return None

    ax, ay = abs(x), abs(y)

    # Strongly horizontal?
    if ax >= DIAG_RATIO * ay:
        return "left" if x > 0 else "right"

    # Strongly vertical?
    if ay >= DIAG_RATIO * ax:
        return "back" if y > 0 else "front"

    # Otherwise diagonal (use signs)
    if y < 0:  # front
        return "front-left" if x > 0 else "front-right"
    else:      # back
        return "back-left" if x > 0 else "back-right"


def is_obstacle_robot(px, py, active_robot, threshold=1.0):
    """Return robot name if (px, py) is within `threshold` of another robot; else None."""
    for robot_name, (rx, ry) in shared.robot_positions.items():
        if robot_name == active_robot:
            continue
        if math.hypot(px - rx, py - ry) <= threshold:
            return robot_name
    return None


# ============================================================================
# Status helpers
# ============================================================================

def _status_blocks_dir(statuses, direction: str) -> bool:
    """Map a status-tuple to a boolean: is that `direction` blocked?"""
    x_status, y_status, frdiag_status, brdiag_status = statuses
    if direction in ("left", "right"):
        return x_status == direction
    if direction in ("front", "back"):
        return y_status == direction
    if direction in ("front-left", "front-right"):
        return frdiag_status == direction
    if direction in ("back-left", "back-right"):
        return brdiag_status == direction
    return False


def is_path_clear(direction, robot_name):
    """
    Check if a particular `direction` is clear for `robot_name`.
    Uses the same filtered view as check_sensors_for_obstacle() so
    occupied_grids.txt is respected here too.
    """
    if direction == "Free":
        return True

    statuses, (robot_hit, robot_dir) = check_sensors_for_obstacle(0.0, 0.0, robot_name)

    # Robot present in that exact sector? Not clear.
    if robot_hit and robot_dir == direction:
        return False

    # Otherwise rely on the filtered statuses (static obstacles already ignored).
    return not _status_blocks_dir(statuses, direction)


# ============================================================================
# Public API
# ============================================================================

def check_sensors_for_obstacle(dx, dy, robot_name):
    """
    Aggregate sensor hits into 8 directions and filter using occupied_grids.txt.

    Returns:
        ( [x_status, y_status, frdiag_status, brdiag_status],
          [robot_name_or_None, robot_dir_or_None] )

    Where:
        x_status        ∈ {"left", "right", "Free"}
        y_status        ∈ {"back", "front", "Free"}
        frdiag_status   ∈ {"front-left", "front-right", "Free"}
        brdiag_status   ∈ {"back-left", "back-right", "Free"}

        robot_dir_or_None can be any of the 8 directions if a robot is seen nearby.
    """
    # Track obstacles seen in each of the 8 directions
    seen = {
        "left": False, "right": False, "front": False, "back": False,
        "front-left": False, "front-right": False, "back-left": False, "back-right": False,
    }

    # Track the nearest robot (if any) and its direction
    robot_name_hit, robot_dir_hit, robot_min_d = None, None, float("inf")

    for pt in _iter_points(robot_name):
        d8 = _direction8_of_point(pt)
        if not d8:
            continue

        # world point for this hit
        rx, ry = shared.robot_positions.get(robot_name, (0.0, 0.0))
        px, py = rx + pt[0], ry + pt[1]

        # 1) robot check
        other_robot = is_obstacle_robot(px, py, robot_name)
        if other_robot:
            d = math.hypot(pt[0], pt[1])
            if d < robot_min_d:
                robot_min_d = d
                robot_name_hit = other_robot
                robot_dir_hit = d8
            # NOTE: robot still counts as an obstacle in that sector:
            seen[d8] = True
            continue

        # 2) static-ignore check from occupied_grids.txt
        if _obstacle_in_occupied_file(px, py):
            # optional debug:
            # print(f"ℹ️ Ignoring obstacle @grid {meters_to_grid(px, py, shared.global_occupancy, shared.MAP_RESOLUTION)} dir={d8}")
            continue

        # 3) finally mark as seen only if not ignored
        seen[d8] = True

    # Compose the 4-tuple (as list) for the first element
    x_status = "left" if seen["left"] else ("right" if seen["right"] else "Free")
    y_status = "back" if seen["back"] else ("front" if seen["front"] else "Free")
    frdiag_status = "front-left" if seen["front-left"] else ("front-right" if seen["front-right"] else "Free")
    brdiag_status = "back-left" if seen["back-left"] else ("back-right" if seen["back-right"] else "Free")

    return [x_status, y_status, frdiag_status, brdiag_status], [robot_name_hit, robot_dir_hit]
