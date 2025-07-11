# sim_app/obstacle_awareness.py
from sim_app.shared import latest_data


def is_path_clear(direction, robot_name):
    """
    direction: 'front', 'back', 'left', or 'right'
    robot_name: 'Rob0' or 'Rob1'
    Returns True if no obstacle is detected within the threshold in that direction.
    """
    signal_map = {
        'back': f"{robot_name}_S300_sensor1",
        'left': f"{robot_name}_S300_sensor2",
        'front': f"{robot_name}_S3001_sensor1",
        'right': f"{robot_name}_S3001_sensor2",
    }
    
    signal_name = signal_map.get(direction)
    if not signal_name or signal_name not in latest_data:
        return True  # If no data, assume clear
    elif direction == "Free":
        return True
    for pt in latest_data[signal_name]:
        _, _, _, dist = pt
        if dist:
            return False
    return True

def check_sensors_for_obstacle(dx, dy, robot_name):
    """
    Checks the robot's sensors for obstacles in four directions and determines blocked movement directions.
    Args:
        dx (float): Desired movement in the X direction.
        dy (float): Desired movement in the Y direction.
        robot_name (str): The name of the robot, used to construct sensor signal names.
    Returns:
        tuple: (x_direction, y_direction)
            x_direction (str): Indicates if movement in the X direction is blocked ('left', 'right', or 'Free').
            y_direction (str): Indicates if movement in the Y direction is blocked ('front', 'back', or 'Free').
    Notes:
        - Uses global variable `latest_data` to access sensor readings.
        - Prints a summary of detected obstacles and blocked directions.
    """
    
    sensor_directions = [
        ('back', f"{robot_name}_S300_sensor1"),
        ('left', f"{robot_name}_S300_sensor2"),
        ('front', f"{robot_name}_S3001_sensor1"),
        ('right', f"{robot_name}_S3001_sensor2"),
    ]

    obstacles = []

    for direction, signal in sensor_directions:
        points = latest_data.get(signal, [])
        detected = None
        for pt in points:
            _, _, _, dist = pt
            if dist:
                detected = direction
                break
        obstacles.append(detected)

    # Determine Y direction block
    if obstacles[0] is not None and obstacles[2] is not None:
        y_direction = 'front' if dy < 0 else 'back' if dy > 0 else "Free"
    elif obstacles[0] is not None:
        y_direction = 'back'
    elif obstacles[2] is not None:
        y_direction = 'front'
    else:
        y_direction = "Free"

    # Determine X direction block
    if obstacles[1] is not None and obstacles[3] is not None:
        x_direction = 'right' if dx < 0 else 'left' if dx > 0 else "Free"
    elif obstacles[1] is not None:
        x_direction = 'left'
    elif obstacles[3] is not None:
        x_direction = 'right'
    else:
        x_direction = "Free"

    print(f"🚨 [{robot_name}] Sensor obstacle check: {obstacles} | X dir: {x_direction}, Y dir: {y_direction}")
    return x_direction, y_direction