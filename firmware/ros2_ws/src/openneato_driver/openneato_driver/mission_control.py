#!/usr/bin/env python3
import rclpy
import json
import os
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import BatteryState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from openneato_driver.coverage_planner import ZoneCoveragePlanner

# Import interfaccia action (vedi nota in docking_server.py)
try:
    from openneato_interfaces.action import DockToBase
except ImportError:
    class DockToBase:
        class Goal: pass

class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        
        # --- Navigazione & Action Clients ---
        self.navigator = BasicNavigator()
        self.docking_client = ActionClient(self, DockToBase, 'dock_to_base')
        
        # --- Planner (Zig-Zag) ---
        # Stride 0.25m = passo laterale tra una passata e l'altra
        self.coverage_planner = ZoneCoveragePlanner(stride=0.25)
        
        # --- Subscribers ---
        self.sub_battery = self.create_subscription(BatteryState, 'battery_state', self.battery_callback, 10)
        self.sub_mission = self.create_subscription(String, 'mission/start', self.mission_start_callback, 10)
        
        # --- Publishers (NUOVO: Controllo Aspirazione) ---
        self.cleaning_pub = self.create_publisher(Bool, 'cleaning/active', 10)
        
        # --- Configurazione e Stato ---
        self.state_file = "mission_dump.json"
        self.ZONES_CONFIG_FILE = "/opt/openneato/web_interface/backend/config.json"
        
        self.battery_level = 100.0
        self.is_mission_active = False
        self.current_waypoints = []
        self.current_waypoint_index = 0
        self.home_pose = None
        
        # --- Timers ---
        # Timer persistenza (10s)
        self.create_timer(10.0, self.save_state)
        # Timer logica principale (1s)
        self.create_timer(1.0, self.control_loop)

        self.get_logger().info("Mission Control Initialized")

        # Check resume all'avvio
        self.check_resume_condition()

    def battery_callback(self, msg):
        # Normalizza percentuale 0-100
        self.battery_level = msg.percentage if msg.percentage > 1.0 else msg.percentage * 100.0

    def mission_start_callback(self, msg):
        self.get_logger().info(f"Received mission request: {msg.data}")
        try:
            data = json.loads(msg.data)
            mission_type = data.get("type", "zone_cleaning")
            
            # Salva la posizione corrente come HOME prima di partire
            # (Nota: qui servirebbe leggere /amcl_pose o /odom, per ora assumiamo 0,0 o l'ultima nota)
            self.get_logger().info("Mission Started. Recording HOME position.")
            # TODO: Sottoscriversi a /amcl_pose per salvare la vera home
            
            self.is_mission_active = True
            self.cleaning_pub.publish(Bool(data=True)) # Accendi aspirazione

            if mission_type == "full_cleaning":
                self.start_full_exploration()
            else:
                self.start_zone_cleaning(data.get("zones", []))
                
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON mission request")

    def start_zone_cleaning(self, zone_ids):
        waypoints = []
        for zone_id in zone_ids:
            z_wp = self.load_zone_coordinates(zone_id)
            if z_wp: waypoints.extend(z_wp)
        
        if waypoints:
            self.current_waypoints = waypoints
            self.navigator.followWaypoints(self.current_waypoints)
        else:
            self.get_logger().warn("No waypoints generated for zones.")
            self.abort_and_dock()

    def start_full_exploration(self):
        self.get_logger().info("Starting FULL HOUSE cleaning (Exploration).")
        # QUI andrà integrato un nodo di esplorazione (es. explore_lite)
        # Per ora, non avendo il pacchetto, possiamo solo loggare.
        # In futuro: self.navigator.start_exploration()
        self.get_logger().warn("Exploration logic not installed yet. Staying idle.")
        # Temporaneo: spegni subito per evitare blocchi
        self.is_mission_active = False
        self.cleaning_pub.publish(Bool(data=False))

    def load_zone_coordinates(self, zone_id):
        """Legge il config, estrae i vertici e usa il planner per generare il percorso."""
        if not os.path.exists(self.ZONES_CONFIG_FILE):
            self.get_logger().error("Zones config file not found.")
            return []
            
        try:
            with open(self.ZONES_CONFIG_FILE, 'r') as f:
                zones = json.load(f)
                
            for zone in zones:
                if zone.get('id') == zone_id:
                    points = zone.get('coordinates', []) 
                    
                    if not points:
                        return []
                    
                    # Estrai tuple (x,y) supportando sia dizionari che liste
                    points_tuple = []
                    for p in points:
                        # Se è un dizionario {'x': 1, 'y': 2}
                        if isinstance(p, dict):
                            points_tuple.append((p['x'], p['y']))
                        # Se è una lista o tupla [1, 2]
                        else:
                            points_tuple.append((p[0], p[1]))
                
                    # Genera serpentina usando la lista pulita di tuple
                    return self.coverage_planner.generate_boustrophedon_path(points_tuple)
                    
        except Exception as e:
            self.get_logger().error(f"Error loading zone {zone_id}: {e}")
        return []
    
    def save_state(self):
        if not self.is_mission_active:
            return

        data = {
            "timestamp": time.time(),
            "waypoints": [self.pose_to_dict(p) for p in self.current_waypoints],
            "current_index": self.current_waypoint_index,
            "zone": "living_room" 
        }
        
        try:
            with open(self.state_file, 'w') as f:
                json.dump(data, f)
            self.get_logger().debug("Stato missione salvato.")
        except Exception as e:
            self.get_logger().error(f"Errore salvataggio stato: {e}")

    def load_state(self):
        if not os.path.exists(self.state_file):
            return None
        try:
            with open(self.state_file, 'r') as f:
                return json.load(f)
        except Exception:
            return None

    def check_resume_condition(self):
        # Se batteria carica e c'è un dump valido
        if self.battery_level > 90.0:
            state = self.load_state()
            if state:
                self.get_logger().info("Condizioni Resume soddisfatte. Caricamento missione...")
                self.resume_mission(state)

    def resume_mission(self, state):
        raw_waypoints = state.get('waypoints', [])
        start_index = state.get('current_index', 0)
        
        if not raw_waypoints or start_index >= len(raw_waypoints):
            self.get_logger().info("Missione completata o vuota nel dump.")
            return

        self.current_waypoints = [self.dict_to_pose(p) for p in raw_waypoints]
        self.current_waypoint_index = start_index
        
        remaining_poses = self.current_waypoints[start_index:]
        
        # Resume: Accendi aspiratore e vai
        self.cleaning_pub.publish(Bool(data=True))
        self.navigator.followWaypoints(remaining_poses)
        self.is_mission_active = True

    def control_loop(self):
        # Watchdog Batteria
        if self.is_mission_active and self.battery_level < 20.0:
            self.get_logger().warn("Batteria critica (<20%). Abort missione e rientro alla base.")
            self.abort_and_dock()
            return

        # Monitoraggio Navigazione
        if self.is_mission_active:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("Missione completata!")
                    
                    # --- AZIONE CRITICA: SPEGNI ASPIRAPOLVERE ---
                    self.cleaning_pub.publish(Bool(data=False))
                    self.get_logger().info("Cleaning Motors DEACTIVATED")
                    
                    # Pulisci file stato
                    if os.path.exists(self.state_file):
                        os.remove(self.state_file)
                        
                    self.get_logger().info("Missione completata. Rientro alla base.")
                    self.return_to_base()
                self.is_mission_active = False

    def abort_and_dock(self):
        self.navigator.cancelTask()
        self.save_state()
        self.is_mission_active = False
        
        # Spegni motori pulizia immediatamente
        self.cleaning_pub.publish(Bool(data=False))
        
        self.get_logger().info("Inviando richiesta al Docking Server...")
        goal_msg = DockToBase.Goal()
        self.docking_client.wait_for_server()
        self.docking_client.send_goal_async(goal_msg)

    def return_to_base(self):
        self.get_logger().info("Inviando richiesta al Docking Server...")
        goal_msg = DockToBase.Goal()
        self.docking_client.wait_for_server()
        self.docking_client.send_goal_async(goal_msg)

    # Helpers serializzazione
    def pose_to_dict(self, pose_stamped):
        return {
            'x': pose_stamped.pose.position.x,
            'y': pose_stamped.pose.position.y,
            'z': pose_stamped.pose.position.z,
            'qx': pose_stamped.pose.orientation.x,
            'qy': pose_stamped.pose.orientation.y,
            'qz': pose_stamped.pose.orientation.z,
            'qw': pose_stamped.pose.orientation.w,
            'frame_id': pose_stamped.header.frame_id
        }

    def dict_to_pose(self, d):
        p = PoseStamped()
        p.header.frame_id = d['frame_id']
        p.pose.position.x = d['x']
        p.pose.position.y = d['y']
        p.pose.position.z = d['z']
        p.pose.orientation.x = d['qx']
        p.pose.orientation.y = d['qy']
        p.pose.orientation.z = d['qz']
        p.pose.orientation.w = d['qw']
        return p

def main():
    rclpy.init()
    node = MissionControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()