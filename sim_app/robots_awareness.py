# sim_app/robots_awareness.py
"""
Robot awareness helpers:
- Load and validate candidate free grid cells from a text file.
- Choose temporary parking spots for blocking robots (diagonal-aware).
- Request a robot to clear a path and return it home after the task.

Notes:
- Robot frame: +x = left, -x = right, +y = back, -y = front.
"""

import asyncio
import math
import os

from sim_app import shared
import sim_app.robot_motion as OmniRobotMotion
from sim_app.robot_controller import OmniRobotController
from sim_app.obstacle_awareness import (
    is_path_clear,                 # kept as-is (may be used elsewhere)
    check_sensors_for_obstacle,
)
from sim_app.sensor_fetch import fetch_sensor_data
from sim_app.astar_env import meters_to_grid  # for grid check


# -----------------------------------------------------------------------------
# Config / globals
# -----------------------------------------------------------------------------

controller = OmniRobotController()

# FREE GRIDS LOADING (only used to validate the candidate)
FREE_GRIDS_CANDIDATES = [
    os.path.join(os.path.dirname(__file__), "free_grids.txt"),
    "/mnt/data/free_grids.txt",
]


# -----------------------------------------------------------------------------
# Free-grid loading & validation
# -----------------------------------------------------------------------------

def _load_free_grids(paths=FREE_GRIDS_CANDIDATES) -> set[tuple[int, int]]:
    """Load free grid cells from a text file into a set of (gx, gy)."""
    free = set()
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
                    free.add((int(gx), int(gy)))
                except Exception:
                    pass
        if free:
            print(f"✅ loaded {len(free)} free cells from: {path}")
            break
    if not free:
        print("⚠️ no free_grids.txt found; every candidate will be rejected.")
    return free


_FREE_GRIDS = _load_free_grids()


def _candidate_is_in_free_file(xm: float, ym: float) -> bool:
    """Convert meters → grid and check membership in the free_grids set."""
    g = shared.global_occupancy
    res = shared.MAP_RESOLUTION
    gx, gy = meters_to_grid(xm, ym, g, res)
    ok = (gx, gy) in _FREE_GRIDS
    if not ok:
        print(f"🚫 candidate ({xm:.3f}, {ym:.3f}) → grid ({gx},{gy}) NOT in free_grids.txt")
    else:
        print(f"✅ candidate ({xm:.3f}, {ym:.3f}) → grid ({gx},{gy}) is allowed by free_grids.txt")
    return ok


# -----------------------------------------------------------------------------
# Direction priority & mapping utilities
# -----------------------------------------------------------------------------

def _priorities_for_block_direction(block_direction: str) -> list[str]:
    """
    Build a priority list of directions to try based on where the *detected robot* is.
    Diagonals are preferred when available.
    """
    if block_direction == "front":
        return ["back-left", "back-right", "left", "right", "back"]
    if block_direction == "back":
        return ["front-left", "front-right", "left", "right", "front"]
    if block_direction == "left":
        return ["back-left", "front-left", "back", "front", "right"]
    if block_direction == "right":
        return ["back-right", "front-right", "back", "front", "left"]
    if block_direction == "front-left":
        return ["back-right", "right", "back", "front-right", "left"]
    if block_direction == "front-right":
        return ["back-left", "left", "back", "front-left", "right"]
    if block_direction == "back-left":
        return ["front-right", "right", "front", "back-right", "left"]
    if block_direction == "back-right":
        return ["front-left", "left", "front", "back-left", "right"]
    # fallback
    return ["back-left", "back-right", "left", "right", "back"]


def _make_candidate(curx: float, cury: float, direction: str, margin: float) -> list[float]:
    """
    Return a (x, y) temp goal offset from (curx, cury) in any of the 8 directions.
    Robot frame reminder: +x = left, -x = right, +y = back, -y = front.
    """
    if direction == "left":
        return [curx + margin, cury]
    if direction == "right":
        return [curx - margin, cury]
    if direction == "front":
        return [curx, cury - margin]
    if direction == "back":
        return [curx, cury + margin]
    if direction == "front-left":
        return [curx + margin, cury - margin]
    if direction == "front-right":
        return [curx - margin, cury - margin]
    if direction == "back-left":
        return [curx + margin, cury + margin]
    if direction == "back-right":
        return [curx - margin, cury + margin]
    # fallback: no movement
    return [curx, cury]


_TRIPLET_MAP = {
    "front-left":  ["front-left", "front", "left"],
    "front-right": ["front-right", "front", "right"],
    "back-left":   ["back-left", "back", "left"],
    "back-right":  ["back-right", "back", "right"],
    "front":       ["front", "front-left", "front-right"],
    "back":        ["back", "back-left", "back-right"],
    "left":        ["left", "front-left", "back-left"],
    "right":       ["right", "front-right", "back-right"],
}

# Small probe vectors per sector (robot frame: +x=left, -x=right, +y=back, -y=front)
_PROBE_VEC = {
    "left": (+0.1, 0.0),
    "right": (-0.1, 0.0),
    "front": (0.0, -0.1),
    "back": (0.0, +0.1),
    "front-left": (+0.1, -0.1),
    "front-right": (-0.1, -0.1),
    "back-left": (+0.1, +0.1),
    "back-right": (-0.1, +0.1),
}


def _dir_status_free(dirs, want_dir: str) -> bool:
    """Check if the requested sector is free given the (x, y, frdiag, brdiag) status tuple."""
    x_status, y_status, frdiag_status, brdiag_status = dirs
    if want_dir == "left":
        return x_status != "left"
    if want_dir == "right":
        return x_status != "right"
    if want_dir == "front":
        return y_status != "front"
    if want_dir == "back":
        return y_status != "back"
    if want_dir == "front-left":
        return frdiag_status != "front-left"
    if want_dir == "front-right":
        return frdiag_status != "front-right"
    if want_dir == "back-left":
        return brdiag_status != "back-left"
    if want_dir == "back-right":
        return brdiag_status != "back-right"
    return True


def _dir_is_free_with_new_api(robot_name: str, test_dx: float, test_dy: float, want_dir: str) -> bool:
    """
    Use sensor API to confirm the sector is free for this robot.
    Free only if that exact sector is clear AND no robot is reported in that sector.
    """
    dirs, robot_info = check_sensors_for_obstacle(test_dx, test_dy, robot_name)
    robot_name_hit, robot_dir_hit = robot_info
    if not _dir_status_free(dirs, want_dir):
        return False
    if robot_name_hit and robot_dir_hit == want_dir:
        return False
    return True


def _dir8_free_triplet(robot_name: str, want_dir: str) -> bool:
    """
    A direction is 'free' iff its own sector AND its two related cardinals/diagonal are free.
    We probe each sector separately to avoid losing info when diagonals get collapsed.
    """
    for d in _TRIPLET_MAP[want_dir]:
        dx, dy = _PROBE_VEC[d]
        if not _dir_is_free_with_new_api(robot_name, dx, dy, d):
            return False
    return True


def _vector_to_dir8(dx: float, dy: float) -> str:
    """
    Map a (dx, dy) motion vector to one of the 8 sectors used by the sensor API.
    Robot frame: +x=left, -x=right, +y=back, -y=front.
    """
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return "front"  # arbitrary; won't be used when near zero
    ax, ay = abs(dx), abs(dy)
    DIAG_RATIO = 1.5  # favor diagonals unless one axis dominates by 1.5x
    if ax >= DIAG_RATIO * ay:
        return "left" if dx > 0 else "right"
    if ay >= DIAG_RATIO * ax:
        return "back" if dy > 0 else "front"
    if dy < 0:
        return "front-left" if dx > 0 else "front-right"
    return "back-left" if dx > 0 else "back-right"


# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------

async def find_nearest_free_spot(current_position, block_direction, toward_goal, blocking_robot):
    """
    Diagonal-aware chooser.

    - Builds a priority list of directions based on where the *detected robot* is.
    - For each candidate direction:
        * build a temp goal at a fixed margin,
        * check that sector is free via the new sensor API,
        * verify the cell is present in free_grids.txt.
    - Returns the first valid [x, y] or None.
    """
    safe_margin = 2.0  # meters
    cx, cy = current_position

    # Priority order to try based on where the *robot* was seen
    try_order = _priorities_for_block_direction(block_direction)

    # (Note) intent_vectors kept for clarity; probing uses _PROBE_VEC
    intent_vectors = {
        "left": (+0.1, 0.0),
        "right": (-0.1, 0.0),
        "front": (0.0, -0.1),
        "back": (0.0, +0.1),
        "front-left": (+0.1, -0.1),
        "front-right": (-0.1, -0.1),
        "back-left": (+0.1, +0.1),
        "back-right": (-0.1, +0.1),
    }

    for d in try_order:
        # 1) sensor clearance in that direction (avoid walking into another robot/obstacle)
        if not _dir8_free_triplet(blocking_robot, d):
            continue

        # 2) build candidate point in that direction
        cand = _make_candidate(cx, cy, d, safe_margin)

        # 3) ensure candidate is in free_grids.txt
        if not _candidate_is_in_free_file(cand[0], cand[1]):
            continue

        print(f"🧭 Candidate via {d}: {cand}")
        return cand

    print(f"⚠️ No valid parking spot found for {blocking_robot} with block_direction={block_direction}")
    return None


async def request_robot_to_clear(sim, toward_goal, active_robot, blocking_robot, block_direction):
    """
    Ask `blocking_robot` to move temporarily to a clear (pre-approved) spot.
    """
    print(f"⛔ Asking {blocking_robot} to clear path for {active_robot}")

    controller = OmniRobotController()
    await controller.init_handles(sim, f"Omni{blocking_robot.lower()}")
    motion = OmniRobotMotion.RobotMotion(sim, controller.wheels)

    pos = shared.robot_positions[blocking_robot]
    temp_goal = await find_nearest_free_spot(pos, block_direction, toward_goal, blocking_robot)

    if not temp_goal:
        print("❌ No valid temp goal found.")
        return False

    shared.temp_parking[blocking_robot] = {
        "home": tuple(pos),
        "park": tuple(temp_goal),
        "by": active_robot,
    }
    print(f"🧭 {blocking_robot} will move temporarily to {temp_goal}")

    if temp_goal:
        print(f"🧭 {blocking_robot} will move temporarily to {temp_goal}")

        REACH_EPS = 0.12
        cause_dir = block_direction  # keep the original cause; may replan using it

        while True:
            # ✅ live pose
            cur = await controller.get_position()
            shared.robot_positions[blocking_robot] = (cur[0], cur[1])

            dx = temp_goal[0] - cur[0]
            dy = temp_goal[1] - cur[1]
            print(f"temp_goal_y = {temp_goal[1]}, cur_y = {cur[1]}, dy = {dy}")

            # Reached?
            if abs(dx) < REACH_EPS and abs(dy) < REACH_EPS:
                await motion.stop()
                shared.robot_status[blocking_robot] = "idle"
                print(f"🅿️ {blocking_robot} parked at {temp_goal}")
                return True

            move_dir8 = _vector_to_dir8(dx, dy)

            # 🔄 live obstacle check in the intended sector (triplet-aware)
            if not _dir8_free_triplet(blocking_robot, move_dir8):
                await motion.stop()
                print(f"⚠️ Path blocked while parking ({move_dir8}). Replanning temp goal...")
                new_goal = await find_nearest_free_spot((cur[0], cur[1]), cause_dir, toward_goal, blocking_robot)
                if not new_goal:
                    print("❌ Could not find an alternative temp goal.")
                    return False
                temp_goal = new_goal
                await asyncio.sleep(0.05)
                continue

            # Otherwise drive toward current temp_goal
            move = (
                "Diagonal"
                if (abs(dx) > 0.05 and abs(dy) > 0.05)
                else ("Horizontal" if abs(dx) >= abs(dy) else "Vertical")
            )
            vx, vy = await motion.set_velocity(dx, dy, move)
            print(f"dx={dx}, dy={dy}, vx={vx}, vy={vy}")

            # Refresh both robots' sensors so clearance decisions stay current
            await fetch_sensor_data(sim, f"{active_robot}_S300_combined_data")
            await fetch_sensor_data(sim, f"{active_robot}_S3001_combined_data")
            await fetch_sensor_data(sim, f"{blocking_robot}_S300_combined_data")
            await fetch_sensor_data(sim, f"{blocking_robot}_S3001_combined_data")

            await asyncio.sleep(0.05)

    # No valid temp goal (not in file) or never cleared
    return False


async def return_parked_robot_after_active_done(sim, active_robot):
    """
    After `active_robot` completes its task, return any robot it parked back to its 'home'.
    """
    if not hasattr(shared, "temp_parking"):
        return

    # Collect entries to return that were parked by this active robot
    to_return = [r for r, st in shared.temp_parking.items() if st.get("by") == active_robot]

    for rob in to_return:
        st = shared.temp_parking.get(rob, {})
        home = st.get("home")
        if not home:
            continue

        ctrl = OmniRobotController()
        await ctrl.init_handles(sim, f"Omni{rob.lower()}")
        motion = OmniRobotMotion.RobotMotion(sim, ctrl.wheels)

        REACH_EPS = 0.12
        while True:
            cur = await ctrl.get_position()
            shared.robot_positions[rob] = (cur[0], cur[1])

            dx = home[0] - cur[0]
            dy = home[1] - cur[1]
            if abs(dx) < REACH_EPS and abs(dy) < REACH_EPS:
                await motion.stop()
                # cleanup this parking record
                try:
                    del shared.temp_parking[rob]
                except KeyError:
                    pass
                break

            move = (
                "Diagonal"
                if (abs(dx) > 0.05 and abs(dy) > 0.05)
                else ("Horizontal" if abs(dx) >= abs(dy) else "Vertical")
            )
            await motion.set_velocity(dx, dy, move)
            await asyncio.sleep(0.05)
