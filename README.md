# OpenNeato

![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E.svg)
![Python 3.12](https://img.shields.io/badge/Python-3.12-3776AB.svg)

**OpenNeato** is an open-source project designed to breathe new life into Neato robotic vacuums (specifically tested on the Neato D7) by replacing the proprietary logic board functions with a modern Single Board Computer (SBC) running **ROS 2 Jazzy**.

This project transforms your old vacuum into a smart, fully autonomous robot capable of advanced mapping, navigation, and web-based control, leveraging the power of the `nav2` stack and a custom Python-based driver.

## Hardware Requirements

To build your own OpenNeato, you will need:

*   **Robot:** Neato XV, Botvac, or D-Series (e.g., Neato D7 Connected).
*   **SBC:** Radxa Zero 3W (recommended) or Raspberry Pi Zero 2 W.
*   **Power:** Buck Converter (Step-down) to convert the robot's battery voltage (approx. 14V-16V) to 5V for the SBC.
*   **Storage:** High-endurance MicroSD Card (16GB+).
*   **Connectivity:** UART connection wires (TX/RX/GND) to interface the SBC with the Neato motherboard.

## Installation

The installation process has been fully automated to ensure a smooth setup on Ubuntu 24.04.

1.  **Clone the Repository:**
    ```bash
    git clone https://github.com/94-psy/OpenNeato.git
    cd OpenNeato
    ```

2.  **Run the Installer:**
    ```bash
    chmod +x installer/install.sh
    sudo ./installer/install.sh
    ```

3.  **Follow the Interactive Menu:**
    The script uses a `whiptail` interface. Select **"Install / Update"** to begin.
    The installer will automatically:
    *   Install system dependencies (ROS 2, Python venv, etc.).
    *   Set up permissions (dialout group).
    *   Deploy the firmware and web interface to `/opt/openneato`.
    *   Build the ROS 2 workspace.
    *   Configure and start `systemd` services.

Once finished, the installer will display the IP address of your robot and the URL for the Web Dashboard.

## How to Update (Deploy Pigro)

When new features or bug fixes are released, updating your robot is simple. The installation script is idempotent and handles updates automatically.

```bash
cd OpenNeato
git pull
sudo ./installer/install.sh
```

Select **"Install / Update"** again. The script will detect the changes, rebuild the firmware if necessary, and restart the services.

## Architecture

OpenNeato is built on a modular architecture:

*   **OpenNeato Core (ROS 2):**
    *   **Driver Node:** Interfaces with the Neato hardware via serial (Lidar, Motors, Sensors).
    *   **Nav2 Stack:** Handles mapping (SLAM), path planning, and autonomous navigation.
    *   **Docking Server:** Custom Action Server for precise docking and charging logic.
    *   **Mission Control:** Manages cleaning queues, state persistence, and battery watchdogs.
*   **Web Interface:**
    *   **Backend:** FastAPI (Python) server that bridges the web UI with ROS 2.
    *   **Frontend:** Responsive HTML/JS dashboard for control and status monitoring.

## Disclaimer

**Use at your own risk.** Modifying your robot's hardware or firmware will void the manufacturer's warranty. This software is provided "as is", without warranty of any kind. The authors are not responsible for any damage to your hardware, data loss, or fires caused by improper handling of Li-Ion batteries or electrical components. Always ensure proper insulation and power regulation when modifying electronics.