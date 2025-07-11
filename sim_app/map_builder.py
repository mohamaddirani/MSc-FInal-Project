import numpy as np
from sim_app.shared import latest_data

# Map dimensions in meters
MAP_WIDTH = 40.0
MAP_HEIGHT = 40.0

def update_grid_with_sensors(grid, resolution, robot_name):
    """
    Updates the occupancy grid with sensor data for a specified robot.
    This function iterates over specific sensor signals associated with the given robot name,
    retrieves the latest sensor data points, and marks the corresponding grid cells as occupied.
    Args:
        grid (np.ndarray): 2D numpy array representing the occupancy grid.
        resolution (float): The resolution of the grid (meters per cell).
        robot_name (str): The name of the robot whose sensor data is used.
    Notes:
        - The function expects a global variable `latest_data` containing sensor readings.
        - The grid is updated in-place, with occupied cells set to 1.
        - The function assumes the existence of global constants `MAP_WIDTH` and `MAP_HEIGHT`.
    """

    for signal in [f"{robot_name}_S300_sensor1", f"{robot_name}_S300_sensor2", f"{robot_name}_S3001_sensor1", f"{robot_name}_S3001_sensor2"]:
        for pt in latest_data.get(signal, []):
            x, y, _, dist = pt
            gx = int((x + MAP_WIDTH / 2) / resolution)
            gy = int((y + MAP_HEIGHT / 2) / resolution)
            if 0 <= gx < grid.shape[1] and 0 <= gy < grid.shape[0]:
                grid[gy][gx] = 1  # flip gy/gx: rows = Y, cols = X

def build_occupancy_grid(robot_name, grid_size=200, cell_resolution=0.2):
    grid = np.zeros((grid_size, grid_size), dtype=np.uint8)

    update_grid_with_sensors(grid, cell_resolution, robot_name)

    return grid
