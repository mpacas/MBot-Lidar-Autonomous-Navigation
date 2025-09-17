# MBot LiDAR Autonomous Navigation

**Autonomous navigation implementation using potential field control and A* path planning for differential-drive robots**

[![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=c%2B%2B&logoColor=white)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3776AB?style=flat&logo=python&logoColor=white)](https://python.org/)

## Overview

This project implements **autonomous navigation algorithms** for the MBot robot platform using 2D LiDAR sensing. The system demonstrates fundamental robotics concepts including reactive obstacle avoidance and deliberative path planning in structured indoor environments.

**Academic Context**: Developed for ROB 102 (Introduction to AI & Programming) at University of Michigan

## 🔧 Implemented Features

### Navigation Algorithms
- **Potential Field Control (Bug Navigation)**: Reactive obstacle avoidance combining attractive goal forces with repulsive obstacle forces
- **A* Path Planning**: Optimal path finding using grid-based search with Manhattan/Euclidean heuristics  
- **LiDAR-based Mapping**: Real-time obstacle detection and occupancy grid generation

### Visualization & Interface
- **Real-time Visualization**: Live display of robot pose, LiDAR data, and planned paths
- **Web App Integration**: Compatible with navigation web application for remote monitoring
- **Map Display**: Visual representation of robot's environmental perception

## 🏗️ System Components

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Web Interface  │────│  Navigation Core │────│   MBot Robot    │
│                 │    │      (C++)       │    │    (LiDAR)      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

**Core Modules:**
- **Bug Navigation**: Reactive control for obstacle avoidance
- **Path Planner**: A* search implementation on occupancy grids
- **Map Processor**: LiDAR data conversion to navigation-ready maps
- **Visualization Server**: Interface for web-based monitoring

## 📋 Project Structure

### Bug Navigation (Project 2)
Implements reactive navigation using potential field methodology:
- Attractive forces pull robot toward goal
- Repulsive forces push robot away from obstacles
- Hybrid state machine handles complex scenarios

### Path Planning (Project 3) 
Implements deliberative navigation using A* search:
- Grid-based environmental representation
- Configurable heuristic functions
- Optimal path generation with obstacle avoidance

## 🚀 Getting Started

### Prerequisites
- MBot robot platform with 2D LiDAR
- Ubuntu/Linux environment
- Basic C++ compilation tools

### Running the Code
```bash
# Clone the repository
git clone https://github.com/mpacas/MBot-Lidar-Autonomous-Navigation.git
cd MBot-Lidar-Autonomous-Navigation

# Compile navigation code
make

# Run visualization server (compatible with web app)
./server
```

### Web Interface
The code includes a server executable compatible with the [ROB 102 navigation web app](https://github.com/rob102-staff/nav-app) for real-time visualization and robot control.

## 📊 Capabilities Demonstrated

### Technical Skills
- **Robotics Fundamentals**: Understanding of mobile robot kinematics and control
- **Algorithm Implementation**: Translation of theoretical concepts to working code
- **Sensor Integration**: Processing real LiDAR data for navigation decisions
- **Software Architecture**: Modular design supporting different navigation approaches

### Real-World Applications
- Indoor autonomous navigation
- Obstacle avoidance in dynamic environments  
- Goal-directed robot motion
- Foundation for warehouse/service robotics

## 🎯 Learning Outcomes

This project demonstrates proficiency in:
- Classical robotics navigation algorithms
- Real-time sensor data processing
- C++ programming for embedded systems
- Integration of perception, planning, and control systems

## 📚 Background & References

**Course Context:**
- [ROB 102 Project 2: Potential Field Control](https://robotics102.github.io/projects/a2.html)
- [ROB 102 Project 3: Path Planning](https://robotics102.github.io/projects/a3.html)
- [Team Project Documentation](https://sites.google.com/umich.edu/rob102team7project02/home)

**Algorithmic Foundations:**
- Bug algorithms for reactive navigation
- A* search for optimal path planning
- Potential field methods for obstacle avoidance

---

**University of Michigan ROB 102** | **Fall 2024**
