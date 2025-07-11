"""

AStar search

author: Ashwin Bose (@atb033)

"""
from sim_app.astar_env import grid_to_meters
from sim_app import shared
class AStar():
    """
    AStar pathfinding algorithm implementation for grid-based environments.
    Attributes:
        agent_dict (dict): Dictionary containing agent information, including start and goal positions.
        admissible_heuristic (callable): Function to compute the heuristic estimate from a state to the goal.
        is_at_goal (callable): Function to check if a given state is the goal for a specific agent.
        get_neighbors (callable): Function to retrieve neighboring states and their move costs.
    Methods:
        reconstruct_path(came_from, current):
            Reconstructs the path from the start node to the current node using the came_from mapping.
        search(agent_name):
            Performs the A* search algorithm for the specified agent.
            Returns the path from start to goal as a list of states if a path is found, otherwise returns False.
    """
    def __init__(self, env):
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def search(self, agent_name):
        """
        low level search 
        """
        initial_state = self.agent_dict[agent_name]["start"]
        step_cost = 1
        
        closed_set = set()
        open_set = {initial_state}

        came_from = {}

        g_score = {} 
        g_score[initial_state] = 0

        f_score = {} 

        f_score[initial_state] = self.admissible_heuristic(initial_state, agent_name)
        while open_set:
            temp_dict = {open_item:f_score.setdefault(open_item, float("inf")) for open_item in open_set}
            current = min(temp_dict, key=temp_dict.get)

            if self.is_at_goal(current, agent_name):
                path = self.reconstruct_path(came_from, current)
                shared.latest_astar_path = [grid_to_meters(*p) for p in path]
                print("\n🚦 A* planned path:")
                for i, p in enumerate(path):
                    print(f"  [{i}] Grid: {p} => Meters: {grid_to_meters(*p)}")
                return path



            open_set -= {current}
            closed_set |= {current}

            neighbor_list = self.get_neighbors(current)

            for (neighbor, move_cost) in neighbor_list:
                if neighbor in closed_set:
                    continue
                
                tentative_g_score = g_score.setdefault(current, float("inf")) + move_cost

                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):
                    continue

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self.admissible_heuristic(neighbor, agent_name)
        return False

