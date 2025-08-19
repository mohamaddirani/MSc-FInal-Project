# sim_app/astar.py
from sim_app import shared

class AStar:
    def __init__(self, env):
        self.env = env
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def search(self, agent_name):
        initial_state = self.agent_dict[agent_name]["start"]
        closed_set = set()
        open_set = {initial_state}
        came_from = {}
        g_score = {initial_state: 0.0}
        f_score = {initial_state: self.admissible_heuristic(initial_state, agent_name)}

        while open_set:
            current = min(open_set, key=lambda n: f_score.setdefault(n, float("inf")))
            if self.is_at_goal(current, agent_name):
                path = self.reconstruct_path(came_from, current)
                # Convert to meters using the environment’s grid/res
                shared.latest_astar_path = [self.env.cell_to_meters(p) for p in path]
                print("\n🚦 A* planned path:")
                for i, p in enumerate(path):
                    print(f"  [{i}] Grid: {p} => Meters: {self.env.cell_to_meters(p)}")
                return path

            open_set.remove(current)
            closed_set.add(current)

            for neighbor, move_cost in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                tentative = g_score[current] + move_cost
                if neighbor not in open_set or tentative < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f_score[neighbor] = tentative + self.admissible_heuristic(neighbor, agent_name)
                    open_set.add(neighbor)
        return False
