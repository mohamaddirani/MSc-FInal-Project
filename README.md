# MSc Final Project: Multi-Robot Path Planning and Coordination

## Overview
This project implements a multi-robot simulation environment for path planning, obstacle avoidance, and robot coordination using Python. It features:
- A* path planning and grid-based map representation
- Dynamic obstacle detection and avoidance
- Robot-to-robot communication for path clearing
- Voice interaction via LLM (Large Language Model) integration
- Visualization tools for grid maps and robot paths

## Directory Structure
```
MSc-FInal-Project/
├── sim_app/                # Main application code
│   ├── main.py              # Entry point for simulation
│   ├── LLM.py               # Voice and LLM interaction logic
│   ├── a_star.py            # A* path planning algorithm implementation
│   ├── astart_env.py        # Environment wrapper for A* algorithm
│   ├── robot_controller.py  # Robot movement and axis alignment logic
│   ├── robots_awareness.py  # Cooperative obstacle handling between robots
│   ├── obstacle_awareness.py # Obstacle detection and direction logic
│   ├── sensor_fetch.py      # Interfaces with SICK S300 sensor data
│   ├── map_builder.py       # Occupancy grid construction from sensor data
│   ├── plotter.py           # Path and grid visualization
│   └── shared.py            # Shared state and configuration
├── RemoteApi/              # Remote API scripts for simulation
├── Plotter/                 # Additional plotting utilities
├── Literatures/             # Reference papers and documentation
├── Reports/                 # Project reports and appendices
└── scenes/                 # Simulation scenes and videos
```
## Key Features
- **Path Planning:** Uses A* algorithm with accurate grid-to-meter conversion.
- **Obstacle Avoidance:** Real-time detection of static and dynamic obstacles, including other robots.
- **Parking Logic:** Robots can request others to clear the path and park in validated free spots.
- **Voice Interaction:** Control robots and query status using voice commands (LLM integration).
- **Visualization:** Interactive HTML grid viewer and export tools for free/occupied cells.

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
- Other dependencies as listed in your environment (e.g., speech_recognition, pyttsx3)

## References
- See `Literatures/` for related research papers.
- See `Reports/` for project documentation and appendices.

## Author
**Mohamad Dirani**  
MSc Robotics and Artificial Intelligence  
University of Hertfordshire  
GitHub: [@mohamaddirani](https://github.com/mohamaddirani)
