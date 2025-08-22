# sim_app/obstacle_awareness.py
import sim_app.shared as shared
import math

THRESHOLD_M = 0.75      # max range to consider "blocking"
DIAG_RATIO  = 1      # how strongly horizontal/vertical must dominate to *avoid* a diagonal

def _iter_points(robot_name):
    for sig in (f"{robot_name}_S300_combined_data",
                f"{robot_name}_S3001_combined_data"):
        for p in shared.latest_data.get(sig, []):
            yield p

def _direction8_of_point(p):
    """
    Classify a laser point (robot frame: +x=left, -x=right, +y=back, -y=front)
    into one of: left/right/front/back/front-left/front-right/back-left/back-right
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
    if y < 0:   # front
        return "front-left"  if x > 0 else "front-right"
    else:       # back
        return "back-left"   if x > 0 else "back-right"

def is_obstacle_robot(px, py, active_robot, threshold=1.0):
    for robot_name, (rx, ry) in shared.robot_positions.items():
        if robot_name == active_robot:
            continue
        if math.hypot(px - rx, py - ry) <= threshold:
            return robot_name
    return None

def is_path_clear(direction, robot_name):
    """
    Kept for backward compatibility with your other code.
    direction can be: "Free", any of the 4 cardinals, or one of the diagonals.
    """
    if direction == "Free":
        return True
    # If a robot is encoded as "RobX", it's not clear
    if isinstance(direction, str) and direction.startswith("Rob"):
        return False
    for pt in _iter_points(robot_name):
        d8 = _direction8_of_point(pt)
        if d8 == direction:
            return False
    return True

def check_sensors_for_obstacle(dx, dy, robot_name):
    """
    NEW RETURN SHAPE:
      (
        [x_status, y_status, frdiag_status, brdiag_status],
        [robot_name_or_None, robot_dir_or_None]
      )

    x_status  ∈ {"left","right","Free"}
    y_status  ∈ {"back","front","Free"}
    frdiag_status ∈ {"front-left","front-right","Free"}
    brdiag_status ∈ {"back-left","back-right","Free"}

    robot_dir_or_None can be any of the 8 directions if a robot is seen nearby.
    """
    # Track obstacles seen in each of the 8 directions
    seen = {
        "left": False, "right": False, "front": False, "back": False,
        "front-left": False, "front-right": False, "back-left": False, "back-right": False
    }

    # Track the nearest robot (if any) and its direction
    robot_name_hit, robot_dir_hit, robot_min_d = None, None, float("inf")

    for pt in _iter_points(robot_name):
        d8 = _direction8_of_point(pt)
        if not d8:
            continue

        # mark generic obstacle in that direction
        seen[d8] = True

        # compute world point for robot check
        rx, ry = shared.robot_positions.get(robot_name, (0.0, 0.0))
        px, py = rx + pt[0], ry + pt[1]

        other_robot = is_obstacle_robot(px, py, robot_name)
        if other_robot:
            d = math.hypot(pt[0], pt[1])
            if d < robot_min_d:
                robot_min_d = d
                robot_name_hit = other_robot
                robot_dir_hit  = d8

    # Compose the 4-tuple (as list) for the first element
    # X status: prefer reporting the side that has an obstacle; if both, pick the nearer conceptually.
    x_status = "left" if seen["left"] else ("right" if seen["right"] else "Free")
    # Y status:
    y_status = "back" if seen["back"] else ("front" if seen["front"] else "Free")
    # Front diagonals:
    frdiag_status = "front-left" if seen["front-left"] else ("front-right" if seen["front-right"] else "Free")
    # Back diagonals:
    brdiag_status = "back-left" if seen["back-left"] else ("back-right" if seen["back-right"] else "Free")

    return [x_status, y_status, frdiag_status, brdiag_status], [robot_name_hit, robot_dir_hit]
