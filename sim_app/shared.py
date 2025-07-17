# sim_app/shared.py
"""
Shared module for storing global state and data structures used across the simulation application.
Attributes:
    robot_name (str or None): Name of the robot, can be set dynamically.
    latest_data (defaultdict): Stores the latest signal data for any signal name, with default as an empty list.
    all_sensor_data (defaultdict): Stores all sensor data for any signal name, with default as an empty list.
    latest_astar_path (list): Stores the most recently computed A* path.
    executed_path (list): Stores the path that has been executed by the robot.
    planned_path (list): Stores the currently planned path for the robot.
    robot_goal (list): The current goal position for the robot, as a list (e.g., [x, y]).
    robot_start (tuple): The starting position of the robot, as a tuple (x, y).
"""
from collections import defaultdict

robot_name = None

# Dynamically stores any signal name with default as empty list
latest_data = defaultdict(list)
all_sensor_data = defaultdict(list) 
latest_astar_path = []
executed_path = [] 
planned_path = []
robot_start = (0.0, 0.0)

# Dictionary to store current goal of each robot
robot_positions = {
    "Rob0": (0.0, 0.0),
    "Rob1": (0.0, 0.0),
    "Rob2": (0.0, 0.0),
}

# Dictionary to store current goal of each robot
robot_goal = {
    "Rob0": None,
    "Rob1": None,
    "Rob2": None
}

# Dictionary to track each robot’s status
robot_status = {
    "Rob0": "idle",
    "Rob1": "idle",
    "Rob2": "idle"
}

# Optional: central message log
message_log = []
