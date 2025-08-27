# MSc Final Project: Multi-Robot Path Planning and Coordination

## Overview
This project is a multi-robot simulation environment for path planning, obstacle avoidance, and robot coordination using Python. It includes:
- A* path planning and grid-based map representation
- Dynamic obstacle detection and avoidance
- Robot-to-robot communication for path clearing and parking
- Voice interaction via LLM (Large Language Model) integration
- Visualization tools for grid maps and robot paths

## Directory Structure
```
MSc-FInal-Project/
├── sim_app/                # Main application code
│   ├── main.py              # Entry point for simulation
│   ├── LLM.py               # Voice and LLM interaction logic
│   ├── astar.py             # A* path planning algorithm implementation (see reference below)
│   ├── astar_env.py         # Environment wrapper for A* algorithm
│   ├── robot_controller.py  # Robot movement and axis alignment logic
│   ├── robots_awareness.py  # Cooperative obstacle handling between robots
│   ├── obstacle_awareness.py # Obstacle detection and direction logic
│   ├── sensor_fetch.py      # Interfaces with SICK S300 sensor data
│   ├── map_builder.py       # Occupancy grid construction from sensor data
│   ├── plotter.py           # Path and grid visualization
│   ├── shared.py            # Shared state and configuration
│   └── npz_to_graph.py      # Grid export and HTML viewer
├── RemoteApi/              # Remote API scripts for simulation
├── Plotter/                # Additional plotting utilities
├── Literatures/            # Reference papers and documentation
├── Reports/                # Project reports and appendices
└── scenes/                 # Simulation scenes and videos
```

## Key Features
- **Path Planning:** Uses A* algorithm with accurate grid-to-meter conversion.
- **Obstacle Avoidance:** Real-time detection of static and dynamic obstacles, including other robots.
- **Parking Logic:** Robots can request others to clear the path and park in validated free spots.
- **Voice Interaction:** Control robots and query status using voice commands (LLM integration).
- **Visualization:** Interactive HTML grid viewer and export tools for free/occupied/filtered cells.

## How to Run
1. Install Python 3.10+ and required packages (see below).
2. Run the main simulation:
   ```bash
   python -m sim_app.main
   ```
3. For LLM/voice features:
   ```bash
   python -m sim_app.LLM
   ```
4. To export grid data:
   ```bash
   python sim_app/npz_to_graph.py
   ```

## Requirements
- Python 3.10+
- numpy
- speech_recognition
- pyttsx3
- Other dependencies as listed in your environment

## References
- See `Literatures/` for related research papers.
- See `Reports/` for project documentation and appendices.
- A* implementation reference: [atb033/multi_agent_path_planning - centralized/cbs/a_star.py](https://github.com/atb033/multi_agent_path_planning/blob/master/centralized/cbs/a_star.py)

## Author
**Mohamad Dirani**  
MSc Robotics and Artificial Intelligence  
University of Hertfordshire  
GitHub: [@mohamaddirani](https://github.com/mohamaddirani)
