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
                 soft_cost_gain: float | None = None,
                 # --- NEW: proximity penalty settings ---
                 proximity_k_cells: int | None = None,   # ring radius in cells
                 proximity_cost_gain: float = 0.0):      # extra cost per ring (>=0)
        self.grid = grid.astype(np.float32, copy=False)
        self.H, self.W = self.grid.shape
        self.res = res if res is not None else DEFAULT_RES
        self.agent_dict = {"robot": {"start": start_grid, "goal": goal_grid}}
        self.block_threshold = float(block_threshold)
        self.soft_cost_gain = soft_cost_gain

        # NEW: precompute a binary "blocked" mask once
        self._blocked = (self.grid >= self.block_threshold).astype(np.uint8)

        # NEW: proximity penalty config
        self.proximity_k = int(proximity_k_cells) if proximity_k_cells else 0
        self.proximity_cost_gain = float(proximity_cost_gain)

    def is_at_goal(self, pos, agent_name):
        return pos == self.agent_dict[agent_name]["goal"]

    def admissible_heuristic(self, pos, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        dx = abs(pos[0] - goal[0]); dy = abs(pos[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    # --- NEW: quick Chebyshev ring scan to nearest blocked cell up to k rings ---
    def _chebyshev_dist_to_blocked(self, x: int, y: int) -> int | None:
        """Return d in [1..k] if a blocked cell exists within Chebyshev distance d; else None."""
        if self.proximity_k <= 0:
            return None
        H, W = self.H, self.W
        for d in range(1, self.proximity_k + 1):
            x0, x1 = max(0, x - d), min(W, x + d + 1)
            y0, y1 = max(0, y - d), min(H, y + d + 1)
            # Check the ring border only (four edges)
            # top & bottom rows
            if self._blocked[y0, x0:x1].any() or self._blocked[y1 - 1, x0:x1].any():
                return d
            # left & right cols (excluding corners already checked)
            if self._blocked[y0 + 1:y1 - 1, x0].any() or self._blocked[y0 + 1:y1 - 1, x1 - 1].any():
                return d
        return None

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

                # base step cost (diagonal vs straight)
                step = math.sqrt(2) if dx and dy else 1.0

                # existing soft cost (if your grid has fractional costs)
                if self.soft_cost_gain is not None:
                    step += self.soft_cost_gain * float(g)

                # NEW: add proximity penalty if near blocked cells (even on binary grids)
                if self.proximity_k > 0 and self.proximity_cost_gain > 0.0:
                    d = self._chebyshev_dist_to_blocked(nx, ny)
                    if d is not None:
                        # closer => larger penalty; farther => smaller
                        # e.g., with k=3: d=1 → 3, d=2 → 2, d=3 → 1
                        ring_weight = (self.proximity_k - d + 1)
                        step += self.proximity_cost_gain * ring_weight

                out.append(((nx, ny), step))
        return out

    def cell_to_meters(self, cell_xy: tuple[int,int]) -> tuple[float,float]:
        cx, cy = cell_xy
        return grid_to_meters(cx, cy, self.grid, self.res)

    def meters_to_cell(self, x: float, y: float) -> tuple[int,int]:
        return meters_to_grid(x, y, self.grid, self.res)
