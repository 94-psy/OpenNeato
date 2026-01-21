#!/bin/bash

# OpenNeato Installer for Ubuntu 24.04 (Radxa Zero 3W)
REPO_URL="https://github.com/94-psy/OpenNeato.git"
INSTALL_DIR="/opt/openneato"

# ROS Distribution (kilted, jazzy, rolling, etc.)
ROS_DISTRO="kilted"
IS_UPDATE=0

check_self_update() {
    # 1. Verifica se 'git' è installato
    if ! command -v git &> /dev/null; then
        echo "Git not found. Installing temporarily for update check..."
        apt-get update -qq && apt-get install -y git -qq
    fi

    # 2. Controlla se la cartella corrente è un repository git valido
    if git rev-parse --is-inside-work-tree &> /dev/null; then
        # 3. È un repo git
        whiptail --title "Update Check" --infobox "Checking for installer updates (Git)..." 8 50
        git fetch origin > /dev/null 2>&1
        
        LOCAL_HASH=$(git rev-parse HEAD)
        REMOTE_HASH=$(git rev-parse origin/main)
        
        if [ "$LOCAL_HASH" != "$REMOTE_HASH" ]; then
            if (whiptail --title "Self Update" --yesno "Trovato aggiornamento dell'installer.\nVuoi aggiornare e riavviare?" 10 60); then
                git pull
                exec "$0" "$@"
            fi
        fi
    else
        # 4. NON è un repo git (es. zip scaricato)
        whiptail --title "Update Check" --infobox "Checking for installer updates (HTTP)..." 8 50
        REMOTE_VERSION=$(curl -sSL https://raw.githubusercontent.com/94-psy/OpenNeato/main/VERSION)
        
        if [ -f "VERSION" ]; then
            LOCAL_VERSION=$(cat VERSION)
            # Rimuove spazi bianchi per confronto sicuro
            LOCAL_VERSION=$(echo "$LOCAL_VERSION" | xargs)
            REMOTE_VERSION=$(echo "$REMOTE_VERSION" | xargs)
            
            if [ "$LOCAL_VERSION" != "$REMOTE_VERSION" ] && [ -n "$REMOTE_VERSION" ]; then
                whiptail --title "Update Available" --msgbox "New version available: $REMOTE_VERSION\nYou are using: $LOCAL_VERSION\n\nPlease download the latest release from GitHub." 10 60
            fi
        fi
    fi
}

check_version() {
    # Legge la versione corrente (appena scaricata nel repo)
    if [ -f "VERSION" ]; then
        NEW_VERSION=$(cat VERSION | xargs)
    else
        NEW_VERSION="0.0.0"
    fi

    # Legge la versione installata (se esiste)
    if [ -f "$INSTALL_DIR/VERSION" ]; then
        CURRENT_VERSION=$(cat "$INSTALL_DIR/VERSION" | xargs)
    else
        CURRENT_VERSION="None"
    fi

    # Confronto
    if [ "$CURRENT_VERSION" != "None" ]; then
        if dpkg --compare-versions "$NEW_VERSION" gt "$CURRENT_VERSION"; then
            # A. UPDATE
            if (whiptail --title "Update Available" --yesno "Trovata nuova versione ($NEW_VERSION). La versione installata è $CURRENT_VERSION.\n\nAggiornare mantenendo i dati utente?" 12 70); then
                IS_UPDATE=1
            else
                exit 0
            fi
        elif dpkg --compare-versions "$NEW_VERSION" lt "$CURRENT_VERSION"; then
            # B. DOWNGRADE
            if ! (whiptail --title "DOWNGRADE WARNING" --yesno --backtitle "ATTENZIONE" "Stai installando una versione più vecchia ($NEW_VERSION) di quella presente ($CURRENT_VERSION).\n\nQuesto potrebbe corrompere la configurazione. Continuare?" 12 70); then
                exit 0
            fi
            IS_UPDATE=1
        else
            # C. SAME VERSION
            if (whiptail --title "Reinstall" --yesno "La versione $CURRENT_VERSION è già installata.\n\nVuoi reinstallare?" 10 60); then
                IS_UPDATE=1
            else
                exit 0
            fi
        fi
    fi
}

check_system_requirements() {
    # Check Root
    if [ "$EUID" -ne 0 ]; then
      echo "Please run as root (sudo ./install.sh)"
      exit 1
    fi

    # Detect Real User (Sudoer)
    REAL_USER="${SUDO_USER:-$USER}"
    REAL_GROUP=$(id -gn "$REAL_USER")
    REAL_HOME=$(getent passwd "$REAL_USER" | cut -d: -f6)

    # Ensure Whiptail exists (for UI)
    if ! command -v whiptail &> /dev/null; then
        echo "Installing required UI tools..."
        apt-get update && apt-get install -y whiptail
    fi
}

setup_repositories() {
    whiptail --title "System Update" --infobox "Configuring apt repositories..." 8 78
    sudo apt install -y software-properties-common curl
    sudo add-apt-repository universe
    
    ROS2_LIST="/etc/apt/sources.list.d/ros2.list"
    DO_UPDATE=1
    
    # Check installed ROS Distro
    INSTALLED_DISTRO=""
    if [ -f "$INSTALL_DIR/ros_distro" ]; then
        INSTALLED_DISTRO=$(cat "$INSTALL_DIR/ros_distro" | xargs)
    fi

    if [ -f "$ROS2_LIST" ]; then
        # Se IS_UPDATE=1 e le distro sono diverse, forziamo l'aggiornamento
        if [ "$IS_UPDATE" -eq 1 ] && [ "$INSTALLED_DISTRO" != "$ROS_DISTRO" ] && [ -n "$INSTALLED_DISTRO" ]; then
             DO_UPDATE=1
        elif ! (whiptail --title "Repository Config" --yesno "ROS 2 repository list found ($ROS2_LIST).\nOverwrite/Update it?" 10 60); then
            DO_UPDATE=0
        fi
    fi

    if [ $DO_UPDATE -eq 1 ]; then
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee "$ROS2_LIST" > /dev/null
    fi
}

install_dependencies() {
    whiptail --title "Dependencies" --infobox "Installing system dependencies..." 8 78
    apt update
    
    PACKAGES=(
        "ros-${ROS_DISTRO}-ros-base"
        "ros-${ROS_DISTRO}-nav2-simple-commander"
        "ros-${ROS_DISTRO}-rosbridge-server"
        "python3-venv"
        "git"
        "build-essential"
        "rsync"
        "fake-hwclock"
    )

    for pkg in "${PACKAGES[@]}"; do
        whiptail --title "Installing Dependencies" --infobox "Installing $pkg..." 8 78
        apt install -y "$pkg"
        EXIT_CODE=$?
        
        if [ $EXIT_CODE -ne 0 ]; then
            if (whiptail --title "Installation Error" --yesno "Package '$pkg' failed to install (Exit Code: $EXIT_CODE).\n\nAbort installation?" 10 60); then
                exit 1
            fi
        fi
    done
    
    # Time Persistence (Fake HW Clock)
    if command -v fake-hwclock &> /dev/null; then
        systemctl enable fake-hwclock
        systemctl start fake-hwclock
        fake-hwclock save
    fi

    # User Setup
    if [ -n "$SUDO_USER" ]; then
        usermod -aG dialout $SUDO_USER
    fi
}

deploy_files() {
    whiptail --title "Deploying" --infobox "Copying files to $INSTALL_DIR..." 8 78
    mkdir -p $INSTALL_DIR
    
    # Backup config if exists (safety first)
    if [ -f "$INSTALL_DIR/web_interface/backend/config.json" ]; then
        cp "$INSTALL_DIR/web_interface/backend/config.json" "$INSTALL_DIR/web_interface/backend/config.json.bak"
    fi

    if [ -d "$PWD/firmware" ]; then
        # Use exclude to protect user generated maps (maps/ folder and .pgm files)
        # We do NOT exclude *.yaml globally to ensure ROS 2 parameter files are updated.
        rsync -av --delete --exclude='maps/' --exclude='*.pgm' "$PWD/firmware" "$INSTALL_DIR/"
    else
        echo "Error: firmware directory not found in $PWD"
        exit 1
    fi

    if [ -d "$PWD/web_interface" ]; then
        # Exclude config.json to preserve user settings
        rsync -av --delete --exclude='config.json' "$PWD/web_interface" "$INSTALL_DIR/"
    else
        echo "Error: web_interface directory not found in $PWD"
        exit 1
    fi

    if [ -f "$PWD/VERSION" ]; then
        cp "$PWD/VERSION" "$INSTALL_DIR/"
    fi

    if [ -f "$PWD/requirements.txt" ]; then
        cp "$PWD/requirements.txt" "$INSTALL_DIR/"
    fi
    
    # Save current ROS Distro for future updates
    echo "$ROS_DISTRO" > "$INSTALL_DIR/ros_distro"

    # Change ownership to real user
    chown -R "$REAL_USER":"$REAL_GROUP" "$INSTALL_DIR"
}

restore_user_data() {
    BACKUP_BASE="$REAL_HOME/openneato_backups"
    
    if [ ! -d "$BACKUP_BASE" ]; then
        return
    fi

    # Costruiamo la lista per whiptail
    OPTIONS=()
    OPTIONS+=("SKIP" "Do not restore any backup" "ON")
    
    # Cerca cartelle backup, ordinate per data (più recenti prima)
    while read -r backup_path; do
        dirname=$(basename "$backup_path")
        OPTIONS+=("$dirname" "Restore from $dirname" "OFF")
    done < <(ls -1td "$BACKUP_BASE"/backup_* 2>/dev/null)

    # Se abbiamo trovato solo SKIP (nessun backup reale), usciamo
    if [ ${#OPTIONS[@]} -le 3 ]; then
        return
    fi

    CHOICE=$(whiptail --title "Restore Backup" --radiolist "Trovati vecchi dati di configurazione. Vuoi ripristinare mappe e zone?" 15 78 5 "${OPTIONS[@]}" 3>&1 1>&2 2>&3)
    
    if [ $? -eq 0 ] && [ "$CHOICE" != "SKIP" ]; then
        whiptail --title "Restoring" --infobox "Restoring data from $CHOICE..." 8 78
        cp -r "$BACKUP_BASE/$CHOICE/." "$INSTALL_DIR/"
        chown -R "$REAL_USER":"$REAL_GROUP" "$INSTALL_DIR"
        whiptail --title "Restore Complete" --msgbox "Ripristino completato da $CHOICE" 8 60
    fi
}

setup_python_env() {
    whiptail --title "Python Setup" --infobox "Setting up virtual environment..." 8 78
    if [ ! -d "$INSTALL_DIR/venv" ]; then
        python3 -m venv --system-site-packages "$INSTALL_DIR/venv"
    fi

    "$INSTALL_DIR/venv/bin/pip" install -r "$INSTALL_DIR/requirements.txt"
    "$INSTALL_DIR/venv/bin/pip" install colcon-common-extensions

    # Sync Package Versions with Single Source of Truth
    if [ -f "VERSION" ]; then
        NEW_VER=$(cat VERSION)
        echo "Syncing ROS package versions to $NEW_VER..."
        find "$INSTALL_DIR/firmware" -name package.xml -exec sed -i "s|<version>.*</version>|<version>$NEW_VER</version>|" {} +
    fi
}

build_firmware() {
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
}

configure_services() {
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
User=$REAL_USER
Group=$REAL_GROUP
# Utilizziamo bash -c per fare il source dell'ambiente
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
User=$REAL_USER
Group=$REAL_GROUP
AmbientCapabilities=CAP_NET_BIND_SERVICE
WorkingDirectory=/opt/openneato/web_interface/backend
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
}

do_install() {
    check_system_requirements
    setup_repositories
    install_dependencies
    deploy_files
    restore_user_data
    setup_python_env
    build_firmware
    configure_services

    # Finale
    IP_ADDR=$(hostname -I | awk '{print $1}')
    ACTION="installed"
    if [ "$IS_UPDATE" -eq 1 ]; then
        ACTION="updated"
    fi
    whiptail --title "Installation Complete" --msgbox "OpenNeato $ACTION successfully!\n\nDashboard URL: http://$IP_ADDR\n\nServices started." 12 60
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
        
        # Handle User Data (Maps & Config)
        if (whiptail --title "User Data" --yesno "Vuoi eliminare anche le mappe e le configurazioni salvate?" 10 60); then
            rm -rf $INSTALL_DIR
            DATA_MSG="All data removed."
        else
            # Backup strategy
            BACKUP_BASE="$REAL_HOME/openneato_backups"
            BACKUP_DIR="$BACKUP_BASE/backup_$(date +%Y%m%d_%H%M%S)"
            mkdir -p "$BACKUP_DIR"
            
            # Preserve config and maps using cp --parents to keep structure
            # We suppress errors in case files don't exist
            if [ -d "$INSTALL_DIR" ]; then
                cd "$INSTALL_DIR" || true
                find . -name "config.json" -type f -exec cp --parents {} "$BACKUP_DIR" \; 2>/dev/null
                find . \( -name "*.yaml" -o -name "*.pgm" \) -type f -exec cp --parents {} "$BACKUP_DIR" \; 2>/dev/null
                cd - > /dev/null || true
            fi
            
            # Fix permissions for user access
            chown -R "$REAL_USER":"$(id -gn "$REAL_USER")" "$BACKUP_BASE"
            
            rm -rf $INSTALL_DIR
            DATA_MSG="User data backed up to:\n$BACKUP_DIR"
        fi
        
        # Remove ROS 2 Repo
        if (whiptail --title "Cleanup" --yesno "Vuoi rimuovere anche i repository apt di ROS 2?" 10 60); then
            rm -f /etc/apt/sources.list.d/ros2.list
        fi
        
        whiptail --title "Uninstall Complete" --msgbox "OpenNeato has been removed.\n$DATA_MSG" 12 70
    fi
}

# --- Execution Start ---

check_system_requirements
check_self_update

# Main Menu
CHOICE=$(whiptail --title "OpenNeato Installer" --menu "Choose an option:" 15 60 2 \
"1" "Install / Update" \
"2" "Uninstall" 3>&1 1>&2 2>&3)

exitstatus=$?
if [ $exitstatus != 0 ]; then
    exit 0
fi

# Chiama la funzione solo se l'utente ha scelto "Install / Update"
case $CHOICE in
    "1") 
        check_version  # Prima controlliamo
        do_install     # Poi installiamo
        ;;
    "2") 
        do_uninstall 
        ;;
esac
