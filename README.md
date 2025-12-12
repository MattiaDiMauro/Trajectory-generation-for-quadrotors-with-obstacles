# Trajectory Generation for Quadrotor UAVs with Obstacles

**Authors:** Di Mauro M., Galluzzi G., Torrijos J.
**Institution:** Politecnico di Milano  
**Course:** Adaptive and Autonomous Aerospace Systems  
**Professor:** Davide Invernizzi  
**Academic Year:** 2024/2025  

## Overview

This MATLAB project develops an autonomous trajectory-generation pipeline for a quadrotor UAV operating in a 3D environment with obstacles. The implementation combines **RRT\*** path planning with **minimum-snap polynomial trajectory optimization** to generate smooth, dynamically feasible trajectories.

Part of the starter code was provided as course material and was subsequently extended, modified, and completed during the project.

## Main Topics

### 3D Environment Mapping
- Construction of 3D occupancy maps
- Obstacle generation
- Obstacle inflation based on UAV geometry

### Path Planning
- RRT\* algorithm with tunable parameters:
  - `GoalBias`
  - `MaxConnectionDistance`
  - `MaxIterations`
  - `MaxNumTreeNodes`

### Trajectory Optimization
- Minimum-snap polynomial trajectory fitting
- Time allocation and segment duration optimization

### Collision Avoidance
- Full-state trajectory validation
- Automatic insertion of intermediate waypoints when collisions are detected

### Dynamic Feasibility
- Velocity and acceleration profile analysis
- Ensures compatibility with UAV constraints

### Control Integration
- Export of polynomial coefficients for real-time flight-control applications

## Key Features

- Smooth trajectories with continuity up to the 4th derivative (snap)  
- Automatic intermediate waypoint generation for improved feasibility  
- Time-optimized trajectories with configurable UAV speed  
- Modular and readable MATLAB structure  
- Complete validation: collision-free + dynamically consistent  

## Requirements

- MATLAB 
- Navigation Toolbox  
- Robotics System Toolbox 

## References

- Course material: Adaptive and Autonomous Aerospace Systems, Prof. Davide Invernizzi, Politecnico di Milano  
- Mellinger, D., & Kumar, V. (2011). *Minimum snap trajectory generation and control for quadrotors.*

## License

Educational use — Politecnico di Milano (2024/2025)
