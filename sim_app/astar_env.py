# sim_app/astar_env.py (excerpt)
import math
import numpy as np
from sim_app.shared import MAP_RESOLUTION as DEFAULT_RES

def meters_to_grid(x: float, y: float, grid: np.ndarray, res: float):
    cx = int(grid.shape[1] // 2 + x / res)
    cy = int(grid.shape[0] // 2 + y / res)
    return (cx, cy)

def grid_to_meters(cx: int, cy: int, grid: np.ndarray, res: float):
    x = (cx - grid.shape[1] // 2) * res
    y = (cy - grid.shape[0] // 2) * res
    return (x, y)

class AStarEnvironment:
    """
    grid: 2D array.
      If using occupancy: 0=free, 1=occupied.
      If using cost: values in [0,1] (0 free, 1 very costly).
    """
    def __init__(self, grid: np.ndarray, start_grid, goal_grid,
                 res: float | None = None,
                 block_threshold: float = 0.99,
                 soft_cost_gain: float | None = None):
        self.grid = grid.astype(np.float32, copy=False)
        self.H, self.W = self.grid.shape
        self.res = res if res is not None else DEFAULT_RES
        self.agent_dict = {"robot": {"start": start_grid, "goal": goal_grid}}
        self.block_threshold = float(block_threshold)
        # if set, cells add extra traversal cost instead of blocking
        self.soft_cost_gain = soft_cost_gain

    def is_at_goal(self, pos, agent_name):
        return pos == self.agent_dict[agent_name]["goal"]

    def admissible_heuristic(self, pos, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        dx = abs(pos[0] - goal[0]); dy = abs(pos[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    def get_neighbors(self, pos):
        dirs = [(-1,0),(1,0),(0,-1),(0,1), (-1,-1),(-1,1),(1,-1),(1,1)]
        out = []
        for dx, dy in dirs:
            nx, ny = pos[0] + dx, pos[1] + dy
            if 0 <= nx < self.W and 0 <= ny < self.H:
                g = self.grid[ny, nx]
                # hard block only if cost >= threshold
                if g >= self.block_threshold:
                    continue
                step = math.sqrt(2) if dx and dy else 1.0
                if self.soft_cost_gain is not None:
                    # soft penalty (keeps path straighter if safe)
                    step += self.soft_cost_gain * float(g)
                out.append(((nx, ny), step))
        return out

    def cell_to_meters(self, cell_xy: tuple[int,int]) -> tuple[float,float]:
        cx, cy = cell_xy
        return grid_to_meters(cx, cy, self.grid, self.res)

    def meters_to_cell(self, x: float, y: float) -> tuple[int,int]:
        return meters_to_grid(x, y, self.grid, self.res)
