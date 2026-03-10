# Trust Evaluation Testbed

![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Simulator-Gazebo-F58113)
![Nav2](https://img.shields.io/badge/Navigation-Nav2-6A0DAD)
![C++](https://img.shields.io/badge/C++-Plugin-00599C?logo=cplusplus&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

A modular ROS2 simulation testbed that evaluates how trustworthy an autonomous robot behaves in shared human environments, measuring Reliability, Comfort, and Safety in real time.

---

## Demo

> Demo GIF coming soon - a screen recording of Gazebo and RViz simulation running with live trust metrics.

---

## What Problem Does This Solve?

Traditional robot performance metrics measure task completion and efficiency. But these fail to capture how humans perceive robots in shared spaces.

A robot that completes its task but cuts too close to a person, or fails unexpectedly without recovery, will be rejected regardless of its technical performance.

This testbed introduces a human-centred evaluation framework that measures trust the way humans actually experience it - dynamically, with decay and recovery over time.

---

## System Architecture

```
ROS2 Humble
├── Gazebo (Simulation)
│   ├── Custom World + Obstacles
│   ├── TurtleBot3 (LiDAR equipped)
│   └── Dynamic Human Actors (C++ Plugin)
│
├── Nav2 Stack
│   ├── AMCL (Localisation)
│   ├── Planner Server
│   └── Controller Server
│
└── Trust Evaluation Layer
    ├── goal_setting_service.py   - Generates random navigation goals
    ├── goal_monitor.py           - Tracks success/failure + distance
    ├── reliability_service.py    - Scores goal completion over time
    ├── comfort_service.py        - Scores proximity to humans/obstacles
    ├── safety_service.py         - Scores near-collisions and impacts
    └── trust_service.py          - Composite weighted trust score
```

---

## Trust Metrics

| Metric | What It Measures | Trigger |
|--------|-----------------|---------|
| Reliability | Goal completion vs failure rate | Per goal outcome |
| Comfort | Robot proximity to humans and obstacles | LiDAR scan in real time |
| Safety | Near-collisions and impacts | LiDAR threshold breach |
| Trust Score | Weighted composite 0 to 100 | Every 1 second |

### Composite Trust Formula

```
Trust = (0.4 x Reliability + 0.3 x Comfort + 0.3 x Safety) / 1.0
```

Weights are configurable by the user to reflect different deployment contexts.

### Decay Logic

Each metric implements a diminishing upper bound. After each failure event, the maximum possible score permanently lowers. This mirrors real human psychology: trust can partially recover through good behaviour, but never fully reset after a violation.

---

## Tech Stack

| Tool | Purpose |
|------|---------|
| ROS2 Humble | Robot middleware and communication |
| Gazebo Classic | 3D physics simulation |
| Nav2 | Autonomous navigation stack |
| AMCL | Adaptive Monte Carlo Localisation |
| SLAM Toolbox | Map generation |
| TurtleBot3 Burger | Robot model with 2D LiDAR |
| Python / rclpy | All trust metric nodes |
| C++ | Custom Gazebo actor movement plugin |
| RViz2 | Real-time visualisation |

---

## Getting Started

### Prerequisites

- Ubuntu 22.04
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- TurtleBot3 packages
- Gazebo Classic

### Installation

```bash
git clone https://github.com/6carty/trust_evaluation_testbed.git
cd trust_evaluation_testbed
colcon build
source install/setup.bash
```

### Run the Simulation

```bash
# Terminal 1 - Launch simulation
ros2 launch trust_sim simulation_launch.py

# Terminal 2 - Start goal setting
ros2 run trust_sim goal_setting_service.py

# Terminal 3 - Start trust metrics
ros2 run trust_sim reliability_service.py
ros2 run trust_sim comfort_service.py
ros2 run trust_sim safety_service.py
ros2 run trust_sim trust_service.py

# Terminal 4 - Monitor composite trust score
ros2 topic echo /metrics/trust_score
```

### Visualise Metrics Live

```bash
rqt_plot /metrics/reliability /metrics/comfort /metrics/safety /metrics/trust_score
```

---

## Project Structure

```
trust_evaluation_testbed/
├── src/
│   └── trust_sim/
│       ├── scripts/
│       │   ├── goal_monitor.py
│       │   ├── goal_setting_service.py
│       │   ├── reliability_service.py
│       │   ├── comfort_service.py
│       │   ├── safety_service.py
│       │   └── trust_service.py
│       ├── plugins/
│       │   └── actor_plugin.cpp
│       ├── maps/
│       └── CMakeLists.txt
├── .gitignore
├── requirements.txt
├── LICENSE
└── README.md
```

---

## Key Results

- All MUST functional requirements achieved (87 functional requirements defined, 11 non-functional)
- Comfort metric accurately tracked real-time proximity decay across all test runs
- Robot-as-carer scenario showed measurably increased comfort violations, validating context-sensitivity
- System completed 10-goal autonomous trials without crashing
- Trust decay and recovery logic behaved as expected across all three metrics

---

## Lessons Learned

- ROS2 topic architecture requires careful namespace management. Debugging TF trees and topic conflicts was the steepest learning curve on this project.
- Modular node design paid off significantly. Being able to test each metric in isolation was critical to delivering the project under hardware constraints.
- Trust is not binary. Implementing decay and recovery functions changed how I think about system state and memory in software.
- Hardware constraints shape software design. The project succeeded through modularity and isolation, not raw compute power.

---

## Future Work

- Reactive actors that respond to robot presence
- User-configurable trust thresholds per deployment context
- Physical TurtleBot3 deployment and real-world testing
- Web dashboard for real-time trust score visualisation
- Integration with advanced planners such as DWB and MPPI
- Human user study to validate scores against perceived trust

---

## Dissertation

Developed as a final year BSc dissertation at the University of Birmingham, School of Computer Science, supervised by Dr. Masoumeh Mansouri.

*"A Trust Based Evaluation Testbed for Autonomous Robot Navigation in Shared Spaces"* - Haydon Carty, 2025

---

## Contact

Haydon Carty

[LinkedIn](https://linkedin.com/in/YOUR_LINKEDIN_HERE) · [GitHub](https://github.com/6carty) · haydoncarty@hotmail.co.uk
