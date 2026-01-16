# OpenNeato

![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)
![ROS 2](https://img.shields.io/badge/ROS_2-22314E.svg)
![Python 3.12](https://img.shields.io/badge/Python-3.12-3776AB.svg)

**OpenNeato** is an open-source project designed to breathe new life into Neato robotic vacuums (specifically tested on the Neato D7) by replacing the proprietary logic board functions with a modern Single Board Computer (SBC) running **ROS 2**.

This project transforms your old vacuum into a smart, fully autonomous robot capable of advanced mapping, navigation, and web-based control, leveraging the power of the `nav2` stack and a custom Python-based driver.

## Hardware Requirements

To build your own OpenNeato, you will need:

*   **Robot:** Neato XV, Botvac, or D-Series (e.g., Neato D7 Connected).
*   **SBC:** Radxa Zero 3W (recommended) or Raspberry Pi Zero 2 W.
*   **Power:** Buck Converter (Step-down) to convert the robot's battery voltage (approx. 14V-16V) to 5V for the SBC.
*   **Storage:** High-endurance MicroSD Card (16GB+).
*   **Connectivity:** UART connection wires (TX/RX/GND) to interface the SBC with the Neato motherboard.
---

## 🚀 Getting Started (Zero to Hero)

### 1. Flash the Operating System
We recommend **Ubuntu 24.04 Server** (Joshua Riek's Rockchip build is currently the most stable for Radxa Zero 3W).

1.  **Download the Image:**
    * [Radxa Zero 3 NPU with Ubuntu 24.04](https://github.com/Qengineering/Radxa-Zero-3-NPU-Ubuntu24) (Look for `Radxa_Zero3_NPU_Ubuntu24.img.xz`).
2.  **Download the loader.bin:**
    * [rkdeveloptool](https://docs.radxa.com/en/zero/zero3/low-level-dev/rkdeveloptool) (Look for `rk356x_spl_loader_ddr1056_v1.12.109_no_check_todly.bin`).
4.  **Flash to eMMC (Radxa Zero 3W):**
    * Hold the **Maskrom button** on the board while plugging it into your PC via USB.
    * Use **BalenaEtcher** for writing on a SD or `rkdeveloptool` to flash the `.img` file directly to the eMMC device.
5.  **eMMC**
    * Prepare the image (extract it)
    * (OPTIONAL) if the image is less than the eMMC, expand it (example for a 32GB eMMC):
    ```bash
    truncate -s 28G Radxa_Zero3_NPU_Ubuntu24.img
    sudo gdisk Radxa_Zero3_NPU_Ubuntu24.img
    ```
    * Press in order `x` (menu expert), `e` (relocate backup data structures to the end of the disk), `w` (write), `Y`
  
      
    * Resize the partition:
    ```bash
    sudo losetup -fP --show Radxa_Zero3_NPU_Ubuntu24.img
    ```
    * Note the result: should be somethink like `/dev/loop0`, now wh have to check which partition number we have to increase (look for an `EXT4` partition and note the number)
    ```bash
    lsblk /dev/loop0
    ```
    * Suppose to use loop0p1
    ```bash
    sudo parted /dev/loop0 resizepart 1 100%
    sudo e2fsck -f -y /dev/loop0p1
    sudo resize2fs /dev/loop0p1
    ```
6.  **WiFi Setup (headless setup)**
    * Mount the image:
    ```bash
    mkdir -p radxa_mount
    sudo mount /dev/loop0p1 radxa_mount
    cd radxa_mount/etc/netplan/
    sudo nano 01-netcfg.yaml
    ```
    * Configure your network by copying and editing this file (you can configure more than one network):
    ```bash
    network:
      version: 2
      renderer: NetworkManager
      ethernets:
        eth0:
          dhcp4: true
          optional: true
      wifis:
        wlan0:
          dhcp4: true
          optional: true
          access-points:
            "WIFI_NAME":
              password: "WIFI_PASSWD"
            "WIFI2":
              password: "passwd2"
    ```
    * Save and exit `CTRL + o` and `CTRL + x`
    * 
    * Exit and umount:
    ```bash
    sudo chmod 600 01-netcfg.yaml
    cd ../../../
    sudo umount radxa_mount
    sudo losetup -D
    ```
7.  **Flashing**
    * Put the Radxa in Maskrom and then flash bin file and than the img (could take time)
    ```bash
    sudo rkdeveloptool db rk356x_spl_loader_ddr1056_v1.12.109_no_check_todly.bin
    sudo rkdeveloptool wl 0 Radxa_Zero3_NPU_Ubuntu24.img
    ```
    * Reboot
    ```bash
    sudo rkdeveloptool rd
    ```
    
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
