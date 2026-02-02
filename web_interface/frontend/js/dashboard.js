// Configurazione
const ROS_URL = 'ws://' + window.location.hostname + ':9090';

// Inizializzazione ROS
const ros = new ROSLIB.Ros({ url: ROS_URL });

ros.on('connection', () => {
    document.getElementById('ros-status').className = 'badge bg-success';
    document.getElementById('ros-status').innerText = 'Connected';
    console.log('Connected to ROS bridge');
});

ros.on('error', (error) => {
    document.getElementById('ros-status').className = 'badge bg-danger';
    document.getElementById('ros-status').innerText = 'Error';
    console.log('Error connecting to ROS:', error);
});

ros.on('close', () => {
    document.getElementById('ros-status').className = 'badge bg-warning text-dark';
    document.getElementById('ros-status').innerText = 'Closed';
});

// --- Subscribers ---

// Batteria
const batterySub = new ROSLIB.Topic({
    ros: ros,
    name: '/battery_state',
    messageType: 'sensor_msgs/BatteryState'
});

batterySub.subscribe((msg) => {
    const pct = Math.round((msg.percentage > 1 ? msg.percentage : msg.percentage * 100));
    const bar = document.getElementById('battery-bar');
    bar.style.width = pct + '%';
    document.getElementById('battery-text').innerText = pct + '%';
    
    // Colore barra
    if(pct < 20) { bar.className = 'progress-bar bg-danger battery-level'; }
    else if(pct < 50) { bar.className = 'progress-bar bg-warning battery-level'; }
    else { bar.className = 'progress-bar bg-success battery-level'; }
});

// Mappa (Visualizzazione)
// Nota: Richiede che map_server stia pubblicando /map
const viewer = new ROS2D.Viewer({
    divID: 'map-canvas',
    width: document.getElementById('map-canvas').clientWidth,
    height: 400,
    background: '#2b2b2b'
});

const gridClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: viewer.scene,
    continuous: true // Aggiorna se la mappa cambia
});

gridClient.on('change', () => {
    viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
    viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
});


// --- Backend API Calls ---

async function fetchStatus() {
    try {
        const res = await fetch('/api/status');
        const data = await res.json();
        document.getElementById('robot-state').innerText = data.current_state;
    } catch(e) {
        console.error("API Error", e);
    }
}

async function loadZones() {
    try {
        const res = await fetch('/api/zones');
        const zones = await res.json();
        const list = document.getElementById('zone-list');
        list.innerHTML = '';
        
        zones.forEach(zone => {
            if(zone.type === 'room') {
                const item = document.createElement('label');
                item.className = 'list-group-item bg-dark text-white border-secondary';
                item.innerHTML = `
                    <input class="form-check-input me-1 zone-checkbox" type="checkbox" value="${zone.id}">
                    ${zone.name}
                `;
                list.appendChild(item);
            }
        });
    } catch(e) {
        console.error("Zone Load Error", e);
    }
}

// --- Buttons ---

document.getElementById('btn-start').onclick = async () => {
    // Raccogli zone selezionate
    const selected = [];
    document.querySelectorAll('.zone-checkbox:checked').forEach(cb => selected.push(cb.value));
    
    await fetch('/api/clean/start', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ zone_ids: selected, suction_power: 80 })
    });
};

document.getElementById('btn-stop').onclick = async () => {
    await fetch('/api/clean/stop', { method: 'POST' });
};

// --- Logs Logic ---
let logInterval = null;

async function fetchLogs() {
    try {
        const res = await fetch('/api/logs');
        const data = await res.json();
        const consoleDiv = document.getElementById('log-console');
        if (data.logs && consoleDiv) {
            consoleDiv.innerText = data.logs.join('\n');
            // Auto-scroll to bottom
            // consoleDiv.scrollTop = consoleDiv.scrollHeight;
        }
    } catch(e) {
        console.error("Log Fetch Error", e);
    }
}

const logModal = document.getElementById('logModal');
if (logModal) {
    logModal.addEventListener('shown.bs.modal', () => {
        fetchLogs();
        // logInterval = setInterval(fetchLogs, 3000);
    });
    logModal.addEventListener('hidden.bs.modal', () => {
        if (logInterval) clearInterval(logInterval);
    });
}
document.getElementById('btn-refresh-logs')?.addEventListener('click', fetchLogs);

// Polling stato ogni 2 secondi
setInterval(fetchStatus, 2000);
loadZones();

// --- Manual Control Logic ---

async function sendVelocity(linear, angular) {
    try {
        await fetch('/api/teleop', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({ linear_x: linear, angular_z: angular })
        });
    } catch(e) {
        console.error("Teleop Error", e);
    }
}

let moveInterval = null;

window.startMove = (type, value) => {
    if (moveInterval) clearInterval(moveInterval);
    
    // Invia comando immediato
    if (type === 'linear') sendVelocity(value, 0.0);
    if (type === 'angular') sendVelocity(0.0, value);

    // Ripeti il comando ogni 200ms per sicurezza (heartbeat)
    moveInterval = setInterval(() => {
        if (type === 'linear') sendVelocity(value, 0.0);
        if (type === 'angular') sendVelocity(0.0, value);
    }, 200);
};

window.stopMove = () => {
    if (moveInterval) clearInterval(moveInterval);
    sendVelocity(0.0, 0.0); // Stop forzato
};

// --- Mapping Logic ---

document.getElementById('btn-save-map').onclick = async () => {
    if(confirm("Save current map? This will overwrite existing maps.")) {
        try {
            const res = await fetch('/api/map/save', { method: 'POST' });
            const data = await res.json();
            alert(data.status);
        } catch(e) {
            alert("Error saving map: " + e);
        }
    }
};

// Supporto tastiera (opzionale)
document.addEventListener('keydown', (e) => {
    if(e.repeat) return;
    switch(e.key) {
        case "ArrowUp": window.startMove('linear', 0.1); break; // 0.1 m/s LIMIT
        case "ArrowDown": window.startMove('linear', -0.1); break;
        case "ArrowLeft": window.startMove('angular', 1.0); break;
        case "ArrowRight": window.startMove('angular', -1.0); break;
    }
});

document.addEventListener('keyup', (e) => {
    switch(e.key) {
        case "ArrowUp": 
        case "ArrowDown": 
        case "ArrowLeft": 
        case "ArrowRight": 
            window.stopMove(); 
            break;
    }
});

// --- System Mode Logic ---

async function setMode(mode) {
    if(!confirm(`Switch to ${mode} mode? The robot will restart.`)) return;
    
    try {
        await fetch(`/api/system/mode/${mode}`, { method: 'POST' });
        alert("System is restarting... page will reload in 15 seconds.");
        setTimeout(() => location.reload(), 15000);
    } catch(e) {
        console.error("Mode switch error", e);
        alert("Error switching mode");
    }
}

// Check mode on load
async function checkMode() {
    try {
        const res = await fetch('/api/system/mode');
        const data = await res.json();
        document.getElementById('current-mode-text').innerText = `Current: ${data.mode.toUpperCase()}`;
        
        // Evidenzia il bottone attivo
        if(data.mode === 'mapping') {
            document.getElementById('btn-mode-map').classList.add('active');
            document.getElementById('btn-mode-nav').classList.remove('active');
        } else {
            document.getElementById('btn-mode-nav').classList.add('active');
            document.getElementById('btn-mode-map').classList.remove('active');
        }
    } catch(e) {}
}

// Aggiungi checkMode al setInterval o all'avvio
checkMode();