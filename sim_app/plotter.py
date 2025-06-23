# sim_app/plotter.py
import matplotlib.pyplot as plt
from sim_app.shared import latest_data

def plot_lidar():
    plt.clf()
    for label, points in latest_data.items():
        if points:
            xs = [pt[0] for pt in points]
            ys = [pt[1] for pt in points]
            plt.scatter(xs, ys, s=2, label=label)
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.pause(0.01)
