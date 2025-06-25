from sim_app.shared import latest_data

OBSTACLE_THRESHOLD = 0.5  # meters

def is_path_clear(direction):
    """
    direction: 'front', 'back', 'left', or 'right'
    Returns True if no obstacle is detected within the threshold in that direction.
    """
    signal_map = {
        'back': "S300_sensor1",
        'left': "S300_sensor2",
        'front': "S3001_sensor1",
        'right': "S3001_sensor2"
    }

    signal_name = signal_map.get(direction)
    if not signal_name or signal_name not in latest_data:
        return True  # If no data, assume it's clear

    for pt in latest_data[signal_name]:
        _, _, _, dist = pt
        if dist < OBSTACLE_THRESHOLD:
            print(f"🚧 Obstacle detected on {direction} at {dist:.2f}m")
            return False
    return True
