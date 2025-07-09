import numpy as np
from sim_app.shared import latest_data

# Map dimensions in meters
MAP_WIDTH = 40.0
MAP_HEIGHT = 40.0

def update_grid_with_sensors(grid, resolution, robot_name):

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
