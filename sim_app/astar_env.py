# sim_app/astar_env.py
import math

GRID_SIZE = 200
CELL_RESOLUTION = 0.2

def meters_to_grid(x, y):
    cx = int(GRID_SIZE // 2 + x / CELL_RESOLUTION)
    cy = int(GRID_SIZE // 2 + y / CELL_RESOLUTION)
    return (cx, cy)

def grid_to_meters(cx, cy):
    x = (cx - GRID_SIZE // 2) * CELL_RESOLUTION
    y = (cy - GRID_SIZE // 2) * CELL_RESOLUTION
    return (x, y)

class AStarEnvironment:
    def __init__(self, grid, start_grid, goal_grid):
        self.occupancy = grid
        self.agent_dict = {
            "robot": {
                "start": start_grid,
                "goal": goal_grid
            }
        }


    def is_at_goal(self, pos, agent_name):
        return pos == self.agent_dict[agent_name]["goal"]

    def admissible_heuristic(self, pos, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        dx = abs(pos[0] - goal[0])
        dy = abs(pos[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


    def get_neighbors(self, pos):
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),       # Cardinal
            (-1, -1), (-1, 1), (1, -1), (1, 1)      # Diagonals
        ]

        neighbors = []
        for dx, dy in directions:
            nx, ny = pos[0] + dx, pos[1] + dy
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                if self.occupancy[ny, nx] == 0:  # Free cell
                    cost = math.sqrt(2) if dx != 0 and dy != 0 else 1.0
                    neighbors.append(((nx, ny), cost))
        return neighbors

