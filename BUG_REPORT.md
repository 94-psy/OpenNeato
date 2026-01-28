# OpenNeato Bug Report

This document outlines the functional problems identified during the code analysis of the OpenNeato repository.

## 1. Critical Integration Failure: Missing Nodes
**Severity:** BLOCKER
**Location:** `firmware/ros2_ws/src/openneato_driver/setup.py`, `firmware/ros2_ws/src/openneato_nav/launch/navigation_launch.py`

**Description:**
The core high-level logic nodes, **MissionControl** (cleaning logic) and **DockingServer** (auto-docking), are completely disconnected from the build and runtime system.
1.  **Missing Entry Points:** `setup.py` does not define entry points for `mission_control` or `docking_server`. As a result, these nodes cannot be run via `ros2 run`.
2.  **Missing Launch Inclusion:** `navigation_launch.py` does not attempt to launch these nodes.
3.  **Consequence:** The robot will not react to "Start Cleaning" commands from the web UI (which publishes to `mission/start`), nor will it ever attempt to dock. The vacuum motors will also never turn on because `MissionControl` is responsible for publishing `cleaning/active`.

## 2. Critical Safety Hazard: Serial Port Concurrency
**Severity:** CRITICAL
**Location:** `firmware/ros2_ws/src/openneato_driver/openneato_driver/driver_node.py`

**Description:**
The driver uses a single `threading.Lock()` (`self.serial_lock`) to synchronize access to the serial port between the `lidar_loop` thread and the `main_loop` (sensor reading/motor control).
1.  **Blocking Duration:** The `lidar_loop` sends `GetLDSScan` and reads ~360 lines of text while holding the lock. At 115200 baud, transferring this amount of data takes approximately **0.6 seconds**.
2.  **Safety Blind Spot:** During this 0.6s window, the `main_loop` cannot acquire the lock to run `read_sensors()`.
    *   **Drop Sensors (Cliff Detection):** The cliff sensors are not checked during this window. If the robot is moving at 0.3m/s, it travels 18cm blind. This is enough to drive off a staircase before the sensor is read.
    *   **Motor Commands:** Incoming `cmd_vel` messages cannot send motor commands until the lock is released, introducing massive latency and control jitter.

## 3. Algorithmic Flaw: Coverage Path Planning
**Severity:** HIGH
**Location:** `firmware/ros2_ws/src/openneato_driver/openneato_driver/coverage_planner.py`

**Description:**
The `ZoneCoveragePlanner` uses a naive Bounding Box approach to generate cleaning paths.
1.  **Issue:** It calculates the minimum and maximum X and Y coordinates of the zone and generates a zig-zag path filling the entire rectangle.
2.  **Consequence:** If a user defines a non-rectangular zone (e.g., an L-shaped room or a diagonal hallway), the planner will generate waypoints in the "void" areas outside the user-defined polygon. The robot will attempt to drive through walls or into no-go zones that lie inside the bounding box.

## 4. Algorithmic Flaw: Docking Logic
**Severity:** HIGH
**Location:** `firmware/ros2_ws/src/openneato_driver/openneato_driver/docking_server.py`

**Description:**
The `find_dock_reflector` function calculates the weighted average of angles for detected reflectors, but it fails to handle the circular nature of angles (0 to 360 degrees).
1.  **Issue:** If two valid reflector points are detected at 5° (0.08 rad) and 355° (6.19 rad), the standard average calculation results in `(5 + 355) / 2 = 180°`.
2.  **Consequence:** Instead of detecting the dock at 0° (Front), the robot calculates the target as 180° (Behind) and will turn completely away from the dock.

## 5. Minor Issues
*   **Mapping Node Blocking:** `mapping_node.py` uses `time.sleep(1.0)` inside the callback handling. This blocks the ROS executor, making the node unresponsive to other events during recovery maneuvers.
*   **Lidar Indexing:** `mapping_node.py` assumes `msg.angle_min` is 0. If the driver configuration changes (e.g., to standard -PI to +PI), the logic for finding the "Right" wall (270 degrees) will break.
