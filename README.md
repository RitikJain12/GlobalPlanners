# Global Planners

A ROS 2 framework for comparing global path planning algorithms for autonomous vehicles. Currently implements **A\*** and **Hybrid A\*** with an interactive RViz2 map editor for defining obstacles at runtime.

![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![C++](https://img.shields.io/badge/language-C%2B%2B14-orange)

---

## Algorithms

### A\*
Grid-based search with discrete heading angles. Fast and deterministic; does not respect vehicle kinematics.

- Configurable orientation resolution (default: 8 directions × 45°)
- Footprint-based collision checking
- Suitable for holonomic robots or coarse planning

### Hybrid A\*
Kinematic-aware planner for Ackermann-steered vehicles. Produces drivable paths that respect steering and velocity constraints.

- Continuous heading (default: 72 bins × 5°)
- Dubins curve integration for smooth terminal segments
- Configurable wheelbase, max steering angle, and velocity limits
- Optional reverse motion

---

## Package Structure

```
src/
├── planner/          # ROS 2 node — parameters, topics, visualization
├── aStar/            # Planning algorithms (A* and Hybrid A*)
├── map/              # Occupancy grid and obstacle management
├── utils/            # Point, Node, Dubins path primitives
└── rviz_map_editor/  # RViz2 plugin for interactive obstacle drawing
```

The libraries (`aStar`, `map`, `utils`) have no ROS dependencies and can be used standalone.

---

## Getting Started

### Prerequisites

- [Docker](https://docs.docker.com/get-docker/) and [VS Code](https://code.visualstudio.com/) with the **Dev Containers** extension

### Build and Run

1. Open the repository in VS Code and reopen in the dev container when prompted.

2. Build the workspace:
   ```bash
   cd /ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

3. Launch the planner with RViz2:
   ```bash
   # Terminal 1 — A* planner
   ros2 launch planner astar_planner.launch.py planner:=AStar

   # Terminal 1 — Hybrid A* planner
   ros2 launch planner astar_planner.launch.py planner:=HybridAStar

   # Terminal 2 — RViz2 visualizer
   ros2 launch rviz_map_editor rviz.launch.py
   ```

4. In RViz2:
   - Use the **Area Select** tool to draw rectangular obstacles on the map
   - Use **2D Goal Pose** to set the goal
   - Use **2D Pose Estimate** to set the start pose
   - The planned path appears on the `/path` topic

---

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Sub | Goal pose for planning |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Sub | Start pose |
| `/add_polygon` | `geometry_msgs/PolygonStamped` | Sub | Add obstacle polygon |
| `/path` | `nav_msgs/Path` | Pub | Planned path |
| `/map` | `nav_msgs/OccupancyGrid` | Pub | Current occupancy grid |
| `/footprint` | `geometry_msgs/PolygonStamped` | Pub | Vehicle footprint |
| `/planner_visualization` | `visualization_msgs/MarkerArray` | Pub | Debug: explored nodes |

---

## Configuration

Parameters are loaded from YAML at launch. Both configs share map settings (20 × 20 m, 0.1 m/cell).

### A\* — `config/astar_params.yaml`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `theta_resolution` | `8.0` | Number of discrete heading bins |
| `xy_tolerance` | `0.1` m | Goal position tolerance |
| `theta_tolerance` | `0.8` rad | Goal heading tolerance |
| `timeout` | `20.0` s | Planning timeout |

### Hybrid A\* — `config/hybrid_a_star.yaml`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `theta_resolution` | `72.0` | Number of discrete heading bins |
| `xy_tolerance` | `0.25` m | Goal position tolerance |
| `theta_tolerance` | `0.45` rad | Goal heading tolerance |
| `timeout` | `40.0` s | Planning timeout |
| `wheelbase` | `2.0` m | Vehicle wheelbase |
| `max_steer` | `40.0` deg | Maximum steering angle |
| `steer_resolution` | `10.0` deg | Steering discretization step |
| `max_velocity` | `0.7` m/s | Maximum forward velocity |
| `allow_reverse` | `false` | Enable reverse motion |

---

## Dev Container

The workspace runs inside a Docker container with X11 forwarding for RViz2. GPU support is available but disabled by default — uncomment the relevant lines in [`.devcontainer/devcontainer.json`](.devcontainer/devcontainer.json) to enable it.
