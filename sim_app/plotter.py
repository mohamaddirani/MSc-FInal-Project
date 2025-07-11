# sim_app/plotter.py
import matplotlib.pyplot as plt

from sim_app.shared import all_sensor_data

def plot_sensor_data(robot_name, filename="sensor_data.png"):
    """
    Plots LiDAR sensor data for a specified robot and saves the plot as an image file.
    Parameters:
        robot_name (str): The name of the robot whose sensor data will be plotted.
        filename (str, optional): The filename for saving the plot image. Defaults to "sensor_data.png".
    The function expects a global dictionary `all_sensor_data` where keys are sensor names and values are lists of (x, y) tuples representing sensor readings.
    Each sensor is plotted with a distinct color and labeled accordingly.
    """

    plt.figure(figsize=(8, 8))

    colors = {
        'S300_sensor1': 'purple',  # back
        'S300_sensor2': 'orange',  # left
        'S3001_sensor1': 'red',    # front
        'S3001_sensor2': 'green',  # right
    }

    for signal_suffix, color in colors.items():
        signal = f"{robot_name}_{signal_suffix}"
        points = all_sensor_data.get(signal, []) 
        xs = [pt[0] for pt in points]
        ys = [pt[1] for pt in points]
        plt.scatter(xs, ys, s=10, label=signal_suffix, color=color)

    plt.title(f"LiDAR Sensor Data: {robot_name}")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid(True)
    plt.legend()
    plt.savefig(filename, dpi=300)
    #print(f"✅ Sensor data saved as: {filename}")


def plot_astar_path(path, start=None, goal=None, filename="astar_path.png"):

    plt.figure(figsize=(8, 8))

    if path:
        px, py = zip(*path)
        plt.plot(px, py, color='blue', linewidth=2, label='A* path')
        plt.scatter(px, py, color='blue', s=30)

    if start:
        plt.scatter(*start, color='green', label='Start', s=100, marker='s')
    if goal:
        plt.scatter(*goal, color='red', label='Goal', s=100, marker='*')

    plt.title("A* Path")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid(True)
    plt.legend()
    plt.savefig(filename, dpi=300)
    #print(f"✅ A* path saved as: {filename}")



def plot_executed_path(path, start=None, goal=None, filename="executed_path.png"):

    plt.figure(figsize=(8, 8))

    if path:
        px, py = zip(*path)
        plt.plot(px, py, color='black', linewidth=2, label='Executed Path')
        plt.scatter(px, py, color='black', s=20)

    if start:
        plt.scatter(*start, color='green', label='Start', s=100, marker='s')
    if goal:
        plt.scatter(*goal, color='red', label='Goal', s=100, marker='*')

    plt.title("Robot Executed Path")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid(True)
    plt.legend()
    plt.savefig(filename, dpi=300)
    #print(f"✅ Executed path saved as: {filename}")

def plot_planned_path(path, start=None, goal=None, filename="planned_path.png"):

    plt.figure(figsize=(8, 8))

    if path:
        px, py = zip(*path)
        plt.plot(px, py, color='black', linewidth=2, label='Planned Path')
        plt.scatter(px, py, color='black', s=20)

    if start:
        plt.scatter(*start, color='green', label='Start', s=100, marker='s')
    if goal:
        plt.scatter(*goal, color='red', label='Goal', s=100, marker='*')

    plt.title("Robot Planned Path")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid(True)
    plt.legend()
    plt.savefig(filename, dpi=300)
    #print(f"✅ Planned path saved as: {filename}")

