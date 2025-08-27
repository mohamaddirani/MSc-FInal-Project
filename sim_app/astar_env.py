# sim_app/astar_env.py
"""
A* planning environment and grid/world conversions.

- meters_to_grid / grid_to_meters: convert between world meters and grid cells.
- AStarEnvironment: wraps a cost/occupancy grid with neighbor expansion,
  soft costs, and an optional proximity penalty to keep paths away from walls.
"""

import math
import numpy as np
from sim_app.shared import MAP_RESOLUTION as DEFAULT_RES


# ============================================================================
# Conversions
# ============================================================================

def meters_to_grid(x: float, y: float, grid: np.ndarray, res: float) -> tuple[int, int]:
    """Convert world (x, y) in meters to integer grid indices (cx, cy)."""
    cx = int(grid.shape[1] // 2 + x / res)
    cy = int(grid.shape[0] // 2 + y / res)
    return cx, cy


def grid_to_meters(cx: int, cy: int, grid: np.ndarray, res: float) -> tuple[float, float]:
    """Convert integer grid indices (cx, cy) to world (x, y) in meters."""
    x = (cx - grid.shape[1] // 2) * res
    y = (cy - grid.shape[0] // 2) * res
    return x, y


# ============================================================================
# A* Environment
# ============================================================================

class AStarEnvironment:
    """
    A* search environment over a 2D grid.

    Args:
        grid: 2D array. If occupancy → 0=free, 1=occupied.
              If cost     → values in [0,1] where 0 is free and 1 is very costly.
        start_grid: (cx, cy) start cell.
        goal_grid:  (cx, cy) goal cell.
        res: meters per cell (defaults to shared.MAP_RESOLUTION).
        block_threshold: cells with value >= threshold are treated as hard obstacles.
        soft_cost_gain: if provided, scales the underlying grid cost into step cost.
        proximity_k_cells: Chebyshev ring radius (in cells) for proximity penalty.
        proximity_cost_gain: extra cost per ring (>=0).

    Notes:
        - Proximity penalty keeps solutions away from walls/obstacles even if
          the base grid is binary. It scans up to `proximity_k_cells` rings
          around each neighbor to find the nearest blocked cell.
    """

    def __init__(
        self,
        grid: np.ndarray,
        start_grid,
        goal_grid,
        res: float | None = None,
        block_threshold: float = 0.99,
        soft_cost_gain: float | None = None,
        # --- Proximity penalty settings ---
        proximity_k_cells: int | None = None,   # ring radius in cells
        proximity_cost_gain: float = 0.0,       # extra cost per ring (>=0)
    ):
        self.grid = grid.astype(np.float32, copy=False)
        self.H, self.W = self.grid.shape
        self.res = res if res is not None else DEFAULT_RES

        self.agent_dict = {"robot": {"start": start_grid, "goal": goal_grid}}
        self.block_threshold = float(block_threshold)
        self.soft_cost_gain = soft_cost_gain

        # Precompute a binary "blocked" mask once
        self._blocked = (self.grid >= self.block_threshold).astype(np.uint8)

        # Proximity penalty config
        self.proximity_k = int(proximity_k_cells) if proximity_k_cells else 0
        self.proximity_cost_gain = float(proximity_cost_gain)

    # ------------------------------------------------------------------ #
    # A* Interface
    # ------------------------------------------------------------------ #

    def is_at_goal(self, pos, agent_name) -> bool:
        """Return True if `pos` equals the agent's goal cell."""
        return pos == self.agent_dict[agent_name]["goal"]

    def admissible_heuristic(self, pos, agent_name) -> float:
        """Octile distance (Chebyshev with diagonal weight)."""
        goal = self.agent_dict[agent_name]["goal"]
        dx = abs(pos[0] - goal[0])
        dy = abs(pos[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    # --- Chebyshev ring scan to nearest blocked cell up to k rings ---
    def _chebyshev_dist_to_blocked(self, x: int, y: int) -> int | None:
        """
        Return d in [1..k] if a blocked cell exists within Chebyshev distance d; else None.
        Scans ring borders only for speed.
        """
        if self.proximity_k <= 0:
            return None
        H, W = self.H, self.W
        for d in range(1, self.proximity_k + 1):
            x0, x1 = max(0, x - d), min(W, x + d + 1)
            y0, y1 = max(0, y - d), min(H, y + d + 1)
            # top & bottom rows
            if self._blocked[y0, x0:x1].any() or self._blocked[y1 - 1, x0:x1].any():
                return d
            # left & right cols (excluding corners already checked)
            if self._blocked[y0 + 1:y1 - 1, x0].any() or self._blocked[y0 + 1:y1 - 1, x1 - 1].any():
                return d
        return None

    def get_neighbors(self, pos):
        """
        8-connected neighbors with step cost:
          - 1.0 for cardinal moves, sqrt(2) for diagonals
          - optional soft cost from grid values
          - optional proximity penalty near blocked cells
        Skips neighbors with grid >= block_threshold.
        """
        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1)]
        out = []
        for dx, dy in dirs:
            nx, ny = pos[0] + dx, pos[1] + dy
            if 0 <= nx < self.W and 0 <= ny < self.H:
                g = self.grid[ny, nx]

                # Hard block if cost >= threshold
                if g >= self.block_threshold:
                    continue

                # Base step cost (diagonal vs straight)
                step = math.sqrt(2) if dx and dy else 1.0

                # Existing soft cost (if your grid has fractional costs)
                if self.soft_cost_gain is not None:
                    step += self.soft_cost_gain * float(g)

                # Proximity penalty if near blocked cells (even on binary grids)
                if self.proximity_k > 0 and self.proximity_cost_gain > 0.0:
                    d = self._chebyshev_dist_to_blocked(nx, ny)
                    if d is not None:
                        # closer => larger penalty; farther => smaller
                        # e.g., with k=3: d=1 → 3, d=2 → 2, d=3 → 1
                        ring_weight = (self.proximity_k - d + 1)
                        step += self.proximity_cost_gain * ring_weight

                out.append(((nx, ny), step))
        return out

    # ------------------------------------------------------------------ #
    # Convenience
    # ------------------------------------------------------------------ #

    def cell_to_meters(self, cell_xy: tuple[int, int]) -> tuple[float, float]:
        """Convert a cell coordinate to meters."""
        cx, cy = cell_xy
        return grid_to_meters(cx, cy, self.grid, self.res)

    def meters_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """Convert meters to a cell coordinate."""
        return meters_to_grid(x, y, self.grid, self.res)
