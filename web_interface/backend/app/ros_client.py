import threading
import time
import rclpy
import json
from std_msgs.msg import String
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState

class RosClient(Node):
    def __init__(self):
        super().__init__('openneato_backend_client')
        
        # Publisher per Stop Emergenza
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mission_pub = self.create_publisher(String, 'mission/start', 10)
        self.get_logger().info("RosClient initialized with mission/start publisher")

        
        # Subscriber per Batteria
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        
        # Stato Interno
        self.battery_level = 0.0
        self.is_cleaning = False
        self.current_state = "IDLE"
        
        # Threading
        self._spin_thread = None
        self._stop_event = threading.Event()

    def battery_callback(self, msg: BatteryState):
        """Aggiorna lo stato della batteria."""
        # Normalizza se necessario (assumiamo 0.0-1.0 o 0-100)
        self.battery_level = msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage

    def start(self):
        """Avvia il thread di spinning ROS 2."""
        self._stop_event.clear()
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        self.get_logger().info("ROS Client Thread Started")

    def stop(self):
        """Ferma il thread."""
        self._stop_event.set()
        if self._spin_thread:
            self._spin_thread.join()

    def _spin(self):
        while rclpy.ok() and not self._stop_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)

    def send_stop_command(self):
        """Invia comando di stop (velocità zero)."""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        self.is_cleaning = False
        self.current_state = "STOPPED"
        self.get_logger().info("Emergency Stop Sent")

    def send_velocity(self, linear: float, angular: float):
        """Pubblica comandi di velocità su /cmd_vel."""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def start_cleaning(self, type: str, zone_ids: list, suction_power: int = 80):
        self.get_logger().info(f"Requesting {type}. Zones: {zone_ids}, Power: {suction_power}%")
        self.is_cleaning = True
        self.current_state = "CLEANING_FULL" if type == "full_cleaning" else f"CLEANING_ZONES ({len(zone_ids)})"

        try:
            # Creazione payload JSON più strutturato
            payload_data = {
                "type": type,
                "zones": zone_ids,
                "power": suction_power
            }
            payload = json.dumps(payload_data)
            
            msg = String()
            msg.data = payload
            
            # Pubblicazione su ROS
            self.mission_pub.publish(msg)
            self.get_logger().info(f"Published mission/start: {payload}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish cleaning command: {e}")
            self.is_cleaning = False
            self.current_state = "ERROR"

# Esempio di funzione helper per avviare il nodo (dipende dall'architettura FastAPI/Uvicorn)
def init_ros_node():
    if not rclpy.ok():
        rclpy.init()
    node = RosClient()
    return node