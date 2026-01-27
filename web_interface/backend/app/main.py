import json
import os
import sys
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import List
from uuid import UUID

import rclpy
from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.encoders import jsonable_encoder
from fastapi.middleware.cors import CORSMiddleware

from app.models import Zone, CleaningRequest, RobotStatus
from app.ros_client import RosClient

app = FastAPI(title="OpenNeato Web Interface")

# Configurazione CORS (utile per sviluppo frontend separato)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global State
ros_client: RosClient = None
CONFIG_FILE = Path("config.json")
LOG_FILE = Path("robot.log")

class StreamToLogger:
    """
    Reindirizza lo stream (stdout/stderr) al logger specificato.
    Gestisce il buffering per garantire che le righe siano loggate complete.
    """
    def __init__(self, logger, level):
        self.logger = logger
        self.level = level
        self.buffer = ""

    def write(self, buf):
        self.buffer += buf
        while '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\n', 1)
            self.logger.log(self.level, line)

    def flush(self):
        if self.buffer:
            self.logger.log(self.level, self.buffer)
            self.buffer = ""

# --- Lifecycle Events ---

@app.on_event("startup")
async def startup_event():
    global ros_client
    
    # 1. Configurazione Logger
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    
    # Rotating File Handler (Max 1MB, 3 Backup)
    file_handler = RotatingFileHandler(LOG_FILE, maxBytes=1*1024*1024, backupCount=3)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    
    # Console Handler (scrive su stdout originale per evitare loop con la redirezione)
    if not isinstance(sys.stdout, StreamToLogger):
        original_stdout = sys.stdout
        console_handler = logging.StreamHandler(original_stdout)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)
        
        # 3. Integrazione ROS: Cattura stdout/stderr (inclusi log ROS)
        sys.stdout = StreamToLogger(logger, logging.INFO)
        sys.stderr = StreamToLogger(logger, logging.ERROR)

    # Inizializza ROS 2 context
    rclpy.init()
    # Crea e avvia il nodo client
    ros_client = RosClient()
    ros_client.start()
    
    # Assicura che il file config esista
    if not CONFIG_FILE.exists():
        with open(CONFIG_FILE, "w") as f:
            json.dump([], f)

@app.on_event("shutdown")
async def shutdown_event():
    global ros_client
    if ros_client:
        ros_client.stop()
        ros_client.destroy_node()
    rclpy.shutdown()

# --- API Endpoints ---

@app.get("/api/logs")
async def get_logs():
    """Restituisce le ultime 100 righe del file di log."""
    if not LOG_FILE.exists():
        return {"logs": []}
    try:
        with open(LOG_FILE, "r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()
            return {"logs": [line.strip() for line in lines[-100:]]}
    except Exception as e:
        return {"logs": [f"Errore lettura log: {str(e)}"]}

@app.get("/api/status", response_model=RobotStatus)
async def get_status():
    if not ros_client:
        raise HTTPException(status_code=503, detail="ROS Client not initialized")
    return RobotStatus(
        battery_level=ros_client.battery_level,
        is_cleaning=ros_client.is_cleaning,
        current_state=ros_client.current_state
    )

@app.post("/api/clean/start")
async def start_cleaning(request: CleaningRequest):
    if not ros_client:
        raise HTTPException(status_code=503, detail="ROS Client not initialized")
    ros_client.start_cleaning(request.zone_ids, request.suction_power)
    return {"status": "Cleaning started"}

@app.post("/api/clean/stop")
async def stop_cleaning():
    if not ros_client:
        raise HTTPException(status_code=503, detail="ROS Client not initialized")
    ros_client.send_stop_command()
    return {"status": "Stop command sent"}

@app.get("/api/zones", response_model=List[Zone])
async def get_zones():
    if not CONFIG_FILE.exists():
        return []
    try:
        with open(CONFIG_FILE, "r") as f:
            data = json.load(f)
            return [Zone(**item) for item in data]
    except json.JSONDecodeError:
        return []

@app.post("/api/zones", response_model=Zone)
async def create_zone(zone: Zone):
    zones_data = []
    if CONFIG_FILE.exists():
        with open(CONFIG_FILE, "r") as f:
            try:
                zones_data = json.load(f)
            except json.JSONDecodeError:
                zones_data = []
    
    # Converti il modello in dict JSON-compatibile (gestisce UUID ed Enum)
    new_zone_data = jsonable_encoder(zone)
    zones_data.append(new_zone_data)
    
    with open(CONFIG_FILE, "w") as f:
        json.dump(zones_data, f, indent=2)
    
    return zone

@app.delete("/api/zones/{zone_id}")
async def delete_zone(zone_id: UUID):
    if not CONFIG_FILE.exists():
        raise HTTPException(status_code=404, detail="Zone not found")
    
    with open(CONFIG_FILE, "r") as f:
        zones_data = json.load(f)
    
    # Filtra via la zona con l'ID specificato
    initial_len = len(zones_data)
    zones_data = [z for z in zones_data if z.get("id") != str(zone_id)]
    
    if len(zones_data) == initial_len:
        raise HTTPException(status_code=404, detail="Zone ID not found")
        
    with open(CONFIG_FILE, "w") as f:
        json.dump(zones_data, f, indent=2)
        
    return {"status": "deleted", "id": str(zone_id)}

# --- Static Files (Frontend) ---
# Mount deve essere l'ultimo per non oscurare le API
frontend_path = Path("../frontend")
# Creiamo la cartella se non esiste per evitare crash in dev
if not frontend_path.exists():
    os.makedirs(frontend_path, exist_ok=True)

app.mount("/", StaticFiles(directory=str(frontend_path), html=True), name="frontend")
