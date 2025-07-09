# sim_app/plotter.py
import matplotlib.pyplot as plt

def plot_lidar(path=None, start=None, goal=None, grid=None):
    plt.close('all') 
    plt.figure()
    if grid is not None:
        plt.imshow(grid, cmap='gray_r', origin='upper')
    if path:
        px, py = zip(*path)
        plt.plot(px, py, color='blue', linewidth=2, label='A* path')
    if start:
        plt.scatter(*start, color='green', label='Start')
    if goal:
        plt.scatter(*goal, color='red', label='Goal')
    plt.title("A* Path Planning Visualization")
    plt.legend()
    plt.grid(True)
    plt.savefig(f"path_{start}_{goal}.png")


