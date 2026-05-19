# Mini SLAM Project Report

## 1. Project Overview

`mini_slam` is a simplified 2D LiDAR SLAM project implemented in ROS2.

The goal of this project is to rebuild the main components of a SLAM system step by step, instead of directly modifying the full `slam_toolbox` source code. The project focuses on understanding and implementing the essential SLAM pipeline in a smaller and more controllable system.

At the current stage, the project has implemented the first component: the SLAM frontend. This frontend receives LiDAR scan data and odometry data, associates them by timestamp, and creates a structured scan representation called `LocalizedScan`.

The report is written incrementally. Only the parts that have already been implemented are documented. Future phases will be added after they are completed.

---

## 2. Motivation and Problem Statement

A full SLAM system such as `slam_toolbox` contains many complex components, including scan matching, pose graph construction, loop closure, graph optimization, map serialization, and localization mode.

Studying the full system directly is difficult because many modules are tightly connected. Therefore, this project follows a simplified implementation approach:

```text
study the concept
  ↓
implement a smaller version
  ↓
test each module independently
  ↓
extend step by step
```

The main problem of this project is:

> How can a robot use 2D LiDAR scans and odometry data to incrementally build a SLAM pipeline?

The first implemented step focuses on preparing the input data for SLAM. A LiDAR scan alone is only a set of range measurements. To be useful for SLAM, each scan must be associated with a robot pose and timestamp.

This leads to the first implementation goal:

```text
LaserScan + Odometry pose + Timestamp
              ↓
          LocalizedScan
```

---

## 3. Reference System: slam_toolbox / Karto

This project is inspired by the architecture of `slam_toolbox` and Karto.

In `slam_toolbox`, the ROS2 layer receives sensor data such as `/scan`, handles TF and odometry information, and passes structured scan data into the Karto SLAM backend.

In Karto, an important concept is `LocalizedRangeScan`.

A `LocalizedRangeScan` is not just raw LiDAR data. It contains:

```text
range readings
pose information
corrected pose
timestamp
sensor information
```

This project implements a simplified version of that idea called `LocalizedScan`.

The simplified `LocalizedScan` currently contains:

```text
2D scan points
odometry pose
estimated pose
scan timestamp
odometry timestamp
scan frame id
```

The purpose is to create a clean data representation that can later be used for mapping, scan matching, and pose graph construction.

---

## 4. Proposed mini_slam Architecture

The planned `mini_slam` architecture follows a modular SLAM pipeline:

```text
/scan + /odom
      ↓
SLAM Frontend
      ↓
LocalizedScan
      ↓
Mapping
      ↓
Scan Matching
      ↓
Pose Correction
      ↓
Map and Trajectory
```

At the current stage, only the first module has been implemented:

```text
/scan + /odom
      ↓
SLAM Frontend
      ↓
LocalizedScan
      ↓
Trajectory debug visualization
```

The current system does not yet build a map. The trajectory is published only for debugging and visualization.

The package is organized into several files:

```text
include/mini_slam/types.hpp
include/mini_slam/scan_processor.hpp
src/scan_processor.cpp
src/mini_slam_node.cpp
CMakeLists.txt
package.xml
```

The current code is divided into three main responsibilities:

| Component | File | Responsibility |
|---|---|---|
| Data types | `types.hpp` | Defines `Point2D`, `Pose2D`, and `LocalizedScan` |
| Sensor processing | `scan_processor.hpp`, `scan_processor.cpp` | Converts scan and odometry messages into internal data |
| ROS2 node | `mini_slam_node.cpp` | Subscribes to topics, synchronizes data, stores scans, publishes trajectory |

---

## 5. Phase 1: SLAM Frontend and LocalizedScan Construction

### 5.1 Objective

Phase 1 implements the SLAM frontend of the `mini_slam` system.

The goal is to receive LiDAR and odometry data, associate them by timestamp, and create a `LocalizedScan`.

Input topics:

| Topic | Message type | Meaning |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR measurement |
| `/odom` | `nav_msgs/msg/Odometry` | Robot pose estimate from odometry |

Output topic:

| Topic | Message type | Meaning |
|---|---|---|
| `/mini_slam/trajectory` | `nav_msgs/msg/Path` | Debug trajectory visualization |

The trajectory output is not the main SLAM result. It is only used to verify that the frontend is receiving odometry and generating pose history correctly.

---

### 5.2 Sensor Data Used

From `/scan`, the system uses:

```text
ranges
angle_min
angle_increment
range_min
range_max
scan timestamp
scan frame_id
```

From `/odom`, the system uses:

```text
position.x
position.y
orientation quaternion
odometry timestamp
odometry frame_id
```

The quaternion from odometry is converted into a 2D yaw angle `theta`, forming:

```text
Pose2D(x, y, theta)
```

---

### 5.3 LaserScan to 2D Points

Each valid LiDAR beam is converted from polar coordinates to a 2D point.

For each range measurement:

```text
r = range value
theta = beam angle
```

The conversion is:

```text
x = r * cos(theta)
y = r * sin(theta)
```

Invalid measurements are ignored, including:

```text
NaN
infinity
ranges smaller than range_min
ranges larger than range_max
```

The output is a list of local 2D points in the LiDAR frame.

---

### 5.4 LocalizedScan Representation

The main data structure produced in Phase 1 is `LocalizedScan`.

Conceptually:

```text
LocalizedScan = scan points + odometry pose + timestamp information
```

It stores:

| Field | Meaning |
|---|---|
| `local_points` | 2D points converted from the LiDAR scan |
| `odom_pose` | pose from odometry |
| `estimated_pose` | current pose estimate, currently equal to odometry pose |
| `stamp` | timestamp of the LiDAR scan |
| `odom_stamp` | timestamp of the odometry pose associated with the scan |
| `frame_id` | frame id of the LiDAR scan |

At this stage:

```text
estimated_pose = odom_pose
```

In later stages, `estimated_pose` will be corrected by scan matching.

---

### 5.5 Timestamp Association

A key part of Phase 1 is synchronizing scan data and odometry data.

Instead of directly using the latest odometry message, the node stores recent odometry messages in a history buffer.

When a new scan arrives, the node searches for the odometry pose whose timestamp is closest to the scan timestamp.

The time difference is:

```text
scan_odom_dt = |scan_time - odom_time|
```

If this difference is larger than the threshold:

```text
max_scan_odom_time_delta
```

the scan is rejected.

This prevents the system from associating a LiDAR scan with an odometry pose from the wrong time.

This is important because incorrect scan-pose association can cause mapping errors in later phases.

---

### 5.6 Current Data Flow

The current Phase 1 data flow is:

```text
/odom
  ↓
extract Pose2D
  ↓
store pose with timestamp in odom history


/scan
  ↓
find nearest odometry pose by timestamp
  ↓
check scan-odom time difference
  ↓
convert scan ranges to 2D local points
  ↓
create LocalizedScan
  ↓
store LocalizedScan
  ↓
publish trajectory for debugging
```

---

### 5.7 Testing

Phase 1 can be tested using TurtleBot3 Gazebo simulation.

The simulation provides:

```text
/scan
/odom
/tf
/cmd_vel
/clock
```

The node is launched with simulation time enabled:

```bash
ros2 run mini_slam mini_slam_node --ros-args -p use_sim_time:=true
```

The frontend is considered working if:

```text
/scan is received
/odom is received
scan_odom_dt is below the threshold
LocalizedScan history increases continuously
/mini_slam/trajectory is published
the trajectory can be visualized in RViz
```

In RViz:

```text
Fixed Frame: odom
Display: Path
Topic: /mini_slam/trajectory
```

---

### 5.8 Current Result

The current implementation successfully provides:

```text
ROS2 node for mini_slam
subscription to /scan
subscription to /odom
LaserScan to 2D point conversion
Odometry to Pose2D conversion
timestamp-based scan-odom association
LocalizedScan construction
LocalizedScan history storage
trajectory debug publishing
```

This completes the first implemented part of the `mini_slam` project: the SLAM frontend.

---

### 5.9 Current Limitations

The current implementation does not yet include:

```text
occupancy grid mapping
ray tracing
scan matching
pose graph
loop closure
graph optimization
```

Other current limitations:

```text
scan points are still local LiDAR-frame points
estimated_pose is still equal to odom_pose
the trajectory depends entirely on odometry
no map is published yet
LocalizedScan history is not yet selected as keyframes
```

---

### 5.10 Next Work

The next phase will be added to this report after it is implemented.

The expected next step is occupancy grid mapping:

```text
LocalizedScan
  ↓
transform local scan points to global frame
  ↓
world coordinate to grid coordinate
  ↓
ray tracing
  ↓
update free and occupied cells
  ↓
publish /mini_slam/map
```

This section will be expanded after Phase 2 is implemented.
