# MSc Final Project: LLM-Guided Multi-Agent Robot Navigation in CoppeliaSim

This repository contains the implementation of a multi-robot navigation system that integrates classical path planning (A\*), real-time obstacle avoidance using simulated LiDAR sensors, and goal assignment driven by a Large Language Model (LLM). The system is built using Python and CoppeliaSim, with asynchronous control logic for coordinating multiple robots in parallel.

## 🔍 Overview

This project showcases:

* A\* path planning on a dynamic occupancy grid
* Real-time obstacle detection and avoidance using SICK S300 sensors
* Robot coordination through cooperative path negotiation
* Natural language-based goal assignment via GPT-4
* Modular design for scalability and extensibility

## 📁 Directory Structure

MSc-FInal-Project/
├── main.py                # Entry point for robot coordination and simulation
├── a_star.py              # A* path planning algorithm implementation
├── astart_env.py          # Environment wrapper for A* algorithm
├── map_builder.py         # Occupancy grid construction from sensor data
├── robot_controller.py    # Robot movement and axis alignment logic
├── sensor_fetch.py        # Interfaces with SICK S300 sensor data
├── path_executor.py       # Executes robot motion paths asynchronously
├── robot_awareness.py     # Cooperative obstacle handling between robots
├── check_nearest_robot.py # Dynamic path clearance for blocked robots
└── README.md              # Project documentation

## ✅ Features

* Multi-robot navigation with independent goals
* Asynchronous movement execution using asyncio
* Real-time feedback control using simulated LiDAR sensors
* Deadlock resolution through path and position awareness
* Modular Python scripts for easy adaptation and debugging

## ⚙️ Requirements

* Python 3.9+
* CoppeliaSim (latest version with ZeroMQ Remote API)
* Python dependencies:

  ```bash
  pip install numpy zmq asyncio matplotlib
  ```

## 🚀 How to Run

1. Open the simulation scene in CoppeliaSim
2. Start the simulation engine
3. Run the main control script:

   ```bash
   python main.py
   ```

Robots will begin navigating to assigned goals with obstacle awareness and cooperative movement.

## 🤖 Sensor Configuration

Each robot is equipped with 4 SICK S300 vision sensors:

* `S3001_sensor1`: Front
* `S3001_sensor2`: Right
* `S300_sensor1`: Back
* `S300_sensor2`: Left

These sensors enable 360° awareness for reactive path adjustment.

## 🧠 LLM Integration

The project includes the capability to interface with an LLM (such as GPT-4) for:

* Interpreting natural language goal input
* Assigning coordinates to robots dynamically
* Supporting human-in-the-loop robot control

## 📌 Future Enhancements

* Add SLAM support for real-world deployment
* Integrate GUI for manual or voice-based goal input
* Replace A\* with learning-based or dynamic planners
* Evaluate performance across complex multi-agent scenarios

## 👤 Author

**Mohamad Dirani**
MSc Robotics and Artificial Intelligence
University of Hertfordshire
GitHub: [@mohamaddirani](https://github.com/mohamaddirani)

## 📄 License

This project is provided for academic purposes only. Contact the author for reuse or collaboration opportunities.
