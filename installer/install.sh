#!/bin/bash

# OpenNeato Installer for Ubuntu 24.04 (Radxa Zero 3W)
REPO_URL="https://github.com/94-psy/OpenNeato.git"
INSTALL_DIR="/opt/openneato"

# ROS Distribution (kilted, jazzy, rolling, etc.)
ROS_DISTRO="kilted"

# Check Root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (sudo ./install.sh)"
  exit 1
fi

# Main Menu
CHOICE=$(whiptail --title "OpenNeato Installer" --menu "Choose an option:" 15 60 2 \
"1" "Install / Update" \
"2" "Uninstall" 3>&1 1>&2 2>&3)

exitstatus=$?
if [ $exitstatus != 0 ]; then
    exit 0
fi

do_install() {
    # 1. System Check & Dependencies
    whiptail --title "System Update" --infobox "Updating apt repositories and installing dependencies..." 8 78
    apt-get update
    apt-get install -y ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-nav2-simple-commander ros-${ROS_DISTRO}-rosbridge-server python3-venv git build-essential rsync
    
    # Time Persistence (Fake HW Clock) for systems without RTC
    # Prevents ROS 2 TF errors due to negative time jumps on boot
    apt-get install -y fake-hwclock
    systemctl enable fake-hwclock
    systemctl start fake-hwclock
    fake-hwclock save

    # 2. User Setup
    if [ -n "$SUDO_USER" ]; then
        usermod -aG dialout $SUDO_USER
    fi

    # 3. Deploy Files
    whiptail --title "Deploying" --infobox "Copying files to $INSTALL_DIR..." 8 78
    mkdir -p $INSTALL_DIR
    
    # Copia cartelle firmware e web_interface dalla directory corrente
    # Assumiamo che lo script sia lanciato dalla root del repo o che $PWD contenga i file
    # Per sicurezza, usiamo la directory in cui risiede lo script come riferimento relativo se necessario,
    # ma la richiesta specifica $PWD (directory corrente dell'utente).
    
    if [ -d "$PWD/firmware" ]; then
        rsync -av --delete "$PWD/firmware" "$INSTALL_DIR/"
    else
        echo "Error: firmware directory not found in $PWD"
        exit 1
    fi

    if [ -d "$PWD/web_interface" ]; then
        rsync -av --delete "$PWD/web_interface" "$INSTALL_DIR/"
    else
        echo "Error: web_interface directory not found in $PWD"
        exit 1
    fi

    if [ -f "$PWD/requirements.txt" ]; then
        cp "$PWD/requirements.txt" "$INSTALL_DIR/"
    fi

    # 4. Python Environment
    whiptail --title "Python Setup" --infobox "Setting up virtual environment..." 8 78
    if [ ! -d "$INSTALL_DIR/venv" ]; then
        python3 -m venv --system-site-packages "$INSTALL_DIR/venv"
    fi

    "$INSTALL_DIR/venv/bin/pip" install -r "$INSTALL_DIR/requirements.txt"
    "$INSTALL_DIR/venv/bin/pip" install colcon-common-extensions

    # 5. Build Firmware
    whiptail --title "Building ROS 2 Workspace" --infobox "Compiling OpenNeato firmware (this may take a while)..." 8 78
    cd "$INSTALL_DIR/firmware/ros2_ws"
    
    # Source Virtual Python environment just created above
    if [ -f "$INSTALL_DIR/venv/bin/activate" ]; then
        source $INSTALL_DIR/venv/bin/activate
    else
        echo "Error: Could not source the python virtual environment $INSTALL_DIR/venv/bin/activate"
        exit 1
    fi

    # Source ROS 2
    if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        source /opt/ros/${ROS_DISTRO}/setup.bash
    else
        echo "Error: ROS 2 ${ROS_DISTRO} not found in /opt/ros/${ROS_DISTRO}"
        exit 1
    fi

    # Build
    colcon build --symlink-install

    # 6. System Configuration
    whiptail --title "System Config" --infobox "Configuring services and udev rules..." 8 78
    
    # Udev
    if [ -f "$PWD/config/udev/99-neato.rules" ]; then
        cp "$PWD/config/udev/99-neato.rules" /etc/udev/rules.d/
        udevadm control --reload-rules && udevadm trigger
    fi

    # Systemd
    # Generate services dynamically to inject ROS_DISTRO
    cat <<EOF > /etc/systemd/system/openneato-core.service
[Unit]
Description=OpenNeato Core ROS 2
After=network.target

[Service]
User=root
# Utilizziamo bash -c per fare il source dell'ambiente ROS 2 e del workspace locale prima del lancio
ExecStart=/bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && source /opt/openneato/firmware/ros2_ws/install/setup.bash && exec /opt/openneato/venv/bin/ros2 launch openneato_nav navigation_launch.py'
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

    cat <<EOF > /etc/systemd/system/openneato-web.service
[Unit]
Description=OpenNeato Web Interface
After=openneato-core.service

[Service]
User=root
WorkingDirectory=/opt/openneato/web_interface/backend
# Source necessario per rclpy
ExecStart=/bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && source /opt/openneato/firmware/ros2_ws/install/setup.bash && exec /opt/openneato/venv/bin/python3 -m uvicorn app.main:app --host 0.0.0.0 --port 80'
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

    systemctl daemon-reload
    systemctl enable openneato-core.service
    systemctl enable openneato-web.service
    systemctl restart openneato-core.service
    systemctl restart openneato-web.service

    # Finale
    IP_ADDR=$(hostname -I | awk '{print $1}')
    whiptail --title "Installation Complete" --msgbox "OpenNeato installed successfully!\n\nDashboard URL: http://$IP_ADDR\n\nServices started." 12 60
}

do_uninstall() {
    whiptail --title "Uninstall" --yesno "Are you sure you want to remove OpenNeato?" 10 60
    if [ $? -eq 0 ]; then
        systemctl stop openneato-core.service
        systemctl stop openneato-web.service
        systemctl disable openneato-core.service
        systemctl disable openneato-web.service
        
        rm /etc/systemd/system/openneato-core.service
        rm /etc/systemd/system/openneato-web.service
        rm /etc/udev/rules.d/99-neato.rules
        systemctl daemon-reload
        
        rm -rf $INSTALL_DIR
        
        whiptail --title "Uninstall" --msgbox "OpenNeato has been removed." 8 40
    fi
}

case $CHOICE in
    "1") do_install ;;
    "2") do_uninstall ;;
esac