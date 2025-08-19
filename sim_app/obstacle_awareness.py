import sim_app.shared as shared
import math

THRESHOLD_M = 1.0  # consider points within this range as "blocking"

def _iter_points(robot_name):
    # Both combined signals have (x,y,z,dist) in the robot's laser_frame
    for sig in (f"{robot_name}_S300_combined_data",
                f"{robot_name}_S3001_combined_data"):
        for p in shared.latest_data.get(sig, []):
            yield p

def _direction_of_point(p):
    x, y, *_rest = p
    dist = _rest[1] if len(_rest) >= 2 else (x*x + y*y) ** 0.5
    if dist is None or dist >= THRESHOLD_M:
        return None

    # robot frame rule: +x left, -x right ; +y back, -y front
    if abs(x) >= abs(y):
        return "right" if x < 0 else "left"
    else:
        return "back" if y > 0 else "front"

def is_path_clear(direction, robot_name):
    # "Free" => clear; specific robot name => NOT clear
    if direction == "Free":
        return True
    if isinstance(direction, str) and direction.startswith("Rob"):
        return False
    for pt in _iter_points(robot_name):
        if _direction_of_point(pt) == direction:
            return False
    return True


def check_sensors_for_obstacle(dx, dy, robot_name):
    """
    Returns tuple (x_dir, y_dir)
    Each can be:
        - "Free"
        - "front"/"back"/"left"/"right"
        - robot name ("Rob1", "Rob2", ...)
    """
    seen = {"left": "Free", "right": "Free", "front": "Free", "back": "Free"}

    for pt in _iter_points(robot_name):
        d = _direction_of_point(pt)
        if not d:
            continue

        # convert pt (local laser coords) to world coords
        rx, ry = shared.robot_positions[robot_name]
        px, py = rx + pt[0], ry + pt[1]

        other_robot = is_obstacle_robot(px, py, robot_name)
        if other_robot:
            print(f"🤖 {robot_name} sees {other_robot} in {d} direction")
            seen[d] = other_robot   # overwrite with robot name
        else:
            seen[d] = d             # normal obstacle

    # Y direction (front/back):
    if isinstance(seen["front"], str) and seen["front"].startswith("Rob"):
        y_dir = seen["front"]
    elif isinstance(seen["back"], str) and seen["back"].startswith("Rob"):
        y_dir = seen["back"]
    elif seen["front"] != "Free" and dy < 0:
        y_dir = seen["front"]
    elif seen["back"] != "Free" and dy > 0:
        y_dir = seen["back"]
    else:
        y_dir = "Free"

    # X direction (left/right):
    if isinstance(seen["left"], str) and seen["left"].startswith("Rob"):
        x_dir = seen["left"]
    elif isinstance(seen["right"], str) and seen["right"].startswith("Rob"):
        x_dir = seen["right"]
    elif seen["left"] != "Free" and dx > 0:
        x_dir = seen["left"]
    elif seen["right"] != "Free" and dx < 0:
        x_dir = seen["right"]
    else:
        x_dir = "Free"

    return x_dir, y_dir

def is_obstacle_robot(px, py, active_robot, threshold=0.6):
    """
    Check if a detected obstacle point belongs to another robot.
    """
    for robot_name, (rx, ry) in shared.robot_positions.items():
        if robot_name == active_robot:
            continue
        dist = math.hypot(px - rx, py - ry)
        if dist <= threshold:
            return robot_name
    return None
