# OpenNeato Bug Report V2

**Status:** Codebase analyzed. Critical issues reported previously are **STILL PRESENT**. No effective changes were detected in the inspected files.

## 1. CRITICAL: Missing Nodes (Unchanged)
**Location:** `firmware/ros2_ws/src/openneato_driver/setup.py`, `firmware/ros2_ws/src/openneato_nav/launch/navigation_launch.py`

**Current State:**
*   `setup.py` still lists only one entry point: `neato_driver`.
*   **MissionControl** and **DockingServer** are NOT registered.
*   `navigation_launch.py` does not launch them.

**Impact:** The robot cannot clean autonomously or dock. The web interface buttons "Start Cleaning" and "Return to Base" will have no effect on the robot's movement.

## 2. CRITICAL: Serial Port Blocking (Unchanged)
**Location:** `firmware/ros2_ws/src/openneato_driver/openneato_driver/driver_node.py`

**Current State:**
*   The `lidar_loop` thread still holds `self.serial_lock` for the entire duration of reading 360 Lidar lines.
*   Loop condition: `while lines_read < 365...` inside `with self.serial_lock:`.

**Impact:**
*   **Blindness:** The robot cannot read cliff sensors or bumpers for ~0.6 seconds every second.
*   **Uncontrollable:** Emergency stop commands or manual teleoperation will lag by up to 0.6 seconds.

## 3. HIGH: Algorithmic Flaws (Unchanged)

### Coverage Planner
**Location:** `firmware/ros2_ws/src/openneato_driver/openneato_driver/coverage_planner.py`
*   **State:** Still uses `x_min, x_max` bounding box logic.
*   **Impact:** Robot will attempt to clean areas outside of non-rectangular zones (e.g., L-shapes).

### Docking Logic
**Location:** `firmware/ros2_ws/src/openneato_driver/openneato_driver/docking_server.py`
*   **State:** Still uses simple weighted average for angles.
*   **Impact:** Docking target calculation wraps around incorrectly (0° + 355° -> 180°), causing the robot to drive away from the dock.

## 4. NEW FINDING: Web Interface "Fire and Forget"
**Location:** `web_interface/backend/app/ros_client.py`

**Observation:**
The `start_cleaning` method publishes to `mission/start` but has no way to verify if the Mission Control node (which is currently missing) actually received the command.
*   **Risk:** The UI will show "Cleaning Started" even if the ROS system is down or the Mission Control node failed to start.
*   **Recommendation:** Switch to a ROS 2 Action Server/Client pattern for mission control to provide immediate feedback (Accepted/Rejected) to the Web UI.

## 5. NEW FINDING: Installer Robustness
**Location:** `installer/install.sh`

**Observation:**
The installer runs `colcon build` but does not verify that the generated artifacts actually contain the expected executables. It assumes `setup.py` is correct.
*   **Risk:** Users will see "Installation Complete" but will end up with a broken system where `ros2 run openneato_driver mission_control` returns "executable not found".
