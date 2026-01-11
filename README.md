# 🚗 Hybrid A* Path Planner (ROS 2)

A ROS 2 global path planning component based on the **Hybrid A\*** algorithm, designed for Ackermann-steered autonomous vehicles.  
The planner computes **collision-free, kinematically feasible paths** on an occupancy grid map while respecting steering and turning constraints.

---

## ✨ Features

- Hybrid A* search in continuous state space *(x, y, yaw)*
- Ackermann bicycle motion model
- Obstacle inflation with distance-transform-based soft costmap
- Kinematically feasible curved and straight paths
- Configurable vehicle and planner parameters
- ROS 2 compliant publishers and subscribers
- Unit-tested planning logic

---

## 🧠 Algorithm Overview

This planner extends classical A* to handle vehicle kinematics by searching in a **continuous pose space**:

- **State space:** `(x, y, yaw)`
- **Motion model:** Forward bicycle model with bounded steering
- **Collision checking:** Inflated binary grid + soft costmap
- **Cost function includes:**
  - Path length
  - Steering effort penalty
  - Obstacle proximity cost
  - Euclidean distance heuristic
- **Termination:** Reaches goal within tolerance or search limit

The final path is post-processed to generate evenly spaced waypoints suitable for downstream controllers.

---

## 🧩 ROS Interfaces

### Subscribed Topics

| Topic | Message Type | Description |
|------|-------------|------------|
| `/map` | `nav_msgs/OccupancyGrid` | Global static map |
| `/odom` | `nav_msgs/Odometry` | Current vehicle pose |
| `/dest_goal` | `geometry_msgs/PoseStamped` | Target goal pose |
| `/pathplan_enable` | `std_msgs/Int32` | Planning trigger |

### Published Topics

| Topic | Message Type | Description |
|------|-------------|------------|
| `/planned_path` | `nav_msgs/Path` | Computed global path |
| `/planner_costmap` | `nav_msgs/OccupancyGrid` | Inflated soft costmap |

---

## ⚙️ Parameters

| Parameter | Description |
|---------|-------------|
| `wheelbase` | Vehicle wheelbase (m) |
| `max_steer_deg` | Maximum steering angle |
| `waypoint_spacing` | Output path resolution |
| `map_inflation` | Obstacle inflation radius |
| `odom_offset` | Offset from rear axle |

---

## 🚀 Installation & Usage

### 1. Clone the repository
```bash
git clone https://github.com/<lin2118>/pathplanner.git
cd pathplanner
