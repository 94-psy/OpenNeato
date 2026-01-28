#!/usr/bin/env python3
# Optimized for High Resolution Mapping (map_resolution: 0.02m)
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import serial
import threading
import time
import math
import struct

from std_msgs.msg import Header
from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import BatteryState, LaserScan, PointCloud2, PointField
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

class NeatoDriver(Node):
    def __init__(self):
        super().__init__('neato_driver')

        # --- Parametri ---
        self.declare_parameter('serial_port', '/dev/ttyNeato') # Aggiornato al symlink udev
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('active_recovery', True)
        self.declare_parameter('carpet_threshold', 800)
        
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.active_recovery = self.get_parameter('active_recovery').get_parameter_value().bool_value
        self.carpet_threshold = self.get_parameter('carpet_threshold').get_parameter_value().integer_value

        # --- Hardware Constants ---
        self.wheelbase = 0.240  # 240mm
        self.max_range = 5.0    # Metri

        # --- Serial Interface ---
        self.ser = None
        self.write_lock = threading.Lock()
        self.latest_sensors = {}
        self.current_scan_data = []
        self.connected = False
        self.emergency_until = 0.0
        self.last_valid_packet_time = time.time()
        self.last_button_state = 0

        # --- Publishers ---
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.hazard_pub = self.create_publisher(PointCloud2, '/hazard_cloud', 10)
        self.floor_pub = self.create_publisher(String, '/floor_type', 10)
        self.diag_pub = self.create_publisher(DiagnosticStatus, '/hazard_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # Safety stop
        self.bumper_pub = self.create_publisher(String, 'sensors/bumper', 10)
        self.button_pub = self.create_publisher(Bool, 'user_interface/button', 10)

        # --- Subscribers ---
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.sound_sub = self.create_subscription(
            Int32,
            '/play_sound',
            self.play_sound_callback,
            10
        )
        # NUOVO: Ascolta se deve pulire
        self.cleaning_sub = self.create_subscription(
            Bool,
            'cleaning/active',
            self.cleaning_callback,
            10
        )

        # --- Timers & Threads ---
        self.create_timer(0.2, self.main_loop)
        self.lidar_thread = threading.Thread(target=self.lidar_loop, daemon=True)
        self.lidar_thread.start()
        self.reader_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.reader_thread.start()

        self.get_logger().info(f'Neato Driver avviato su {self.serial_port} @ {self.baud_rate}')

    def connect_serial(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            
            self.ser = serial.Serial(
                self.serial_port, 
                self.baud_rate, 
                timeout=0.1
            )
            self.send_command("TestMode On")
            self.send_command("SetLDSRotation On")
            
            self.connected = True
            self.get_logger().info('Connessione seriale stabilita.')
            self.last_valid_packet_time = time.time()
            return True
        except serial.SerialException as e:
            self.connected = False
            self.get_logger().error(f'Errore connessione seriale: {e}')
            return False

    def send_command(self, cmd):
        if not self.connected or not self.ser:
            return None
        with self.write_lock:
            try:
                self.ser.write(f"{cmd}\n".encode('utf-8'))
                time.sleep(0.01)
            except serial.SerialException:
                self.connected = False
                return None
        return True

    # --- NUOVO: Gestione Motori Pulizia ---
    def cleaning_callback(self, msg):
        """Attiva o disattiva i motori di aspirazione e spazzole."""
        if not self.connected:
            return
            
        if msg.data:
            self.get_logger().info("CLEANING START: Attivazione motori aspirazione.")
            # Accende Aspirazione, Spazzola Principale e Spazzola Laterale
            # Nota: 'VacuumOn' usa parametri di default. Se servisse potenza specifica: SetMotor VacuumSpeed 90
            self.send_command("SetMotor VacuumOn Brush RPM 1000 SideBrushOn")
        else:
            self.get_logger().info("CLEANING STOP: Spegnimento motori.")
            self.send_command("SetMotor VacuumOff BrushDisable SideBrushOff")

    def main_loop(self):
        # Watchdog
        if self.connected and (time.time() - self.last_valid_packet_time > 4.0):
            self.get_logger().error("WATCHDOG: Serial Communication Lost")
            self.handle_serial_failure()
            return

        if not self.connected:
            self.connect_serial()
            return

        self.read_sensors()

    def handle_serial_failure(self):
        diag = DiagnosticStatus()
        diag.level = DiagnosticStatus.ERROR
        diag.name = "Serial Connection"
        diag.message = "Serial Communication Lost"
        self.diag_pub.publish(diag)
        
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
        self.connected = False
        self.send_command("SetMotor LWheelDist 0 RWheelDist 0 Speed 0") # Tenta stop fisico
        self.connect_serial()

    def read_sensors(self):
        if not self.connected: 
            return

        # Invia richieste (la lettura avviene in read_serial_loop)
        self.send_command("GetAnalogSensors")
        self.send_command("GetDigitalSensors")
        
        # Usa i dati più recenti
        sensors = self.latest_sensors

        try:
            # Pubblica Batteria
            if 'BatteryVoltageInmV' in sensors and 'BatteryLevel' in sensors:
                msg = BatteryState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.voltage = sensors['BatteryVoltageInmV'] / 1000.0
                msg.percentage = sensors['BatteryLevel'] / 100.0
                msg.present = True
                self.battery_pub.publish(msg)

            # Safety Reflex
            drop_l = sensors.get('DropSensorLeft', 0)
            drop_r = sensors.get('DropSensorRight', 0)
            mag_l = sensors.get('MagSensorLeft', 0)
            mag_r = sensors.get('MagSensorRight', 0)

            if (drop_l > 40 or drop_r > 40 or mag_l > 0 or mag_r > 0):
                self.trigger_safety_reflex()
            
            # --- Logica Bumper ---
            l_side = int(sensors.get('SNSR_LSIDEBIT', 0))
            r_side = int(sensors.get('SNSR_RSIDEBIT', 0))
            l_front = int(sensors.get('SNSR_LFRONTBIT', 0))
            r_front = int(sensors.get('SNSR_RFRONTBIT', 0))
            
            bumper_msg = ""
            hazards = []
            
            if l_front and r_front:
                bumper_msg = "CENTER"
                hazards = [(0.32, 0.0, 0.0), (0.32, 0.05, 0.0), (0.32, -0.05, 0.0)]
            elif l_front:
                bumper_msg = "FRONT_LEFT"
                hazards = [(0.28, 0.12, 0.0), (0.25, 0.15, 0.0)]
            elif r_front:
                bumper_msg = "FRONT_RIGHT"
                hazards = [(0.28, -0.12, 0.0), (0.25, -0.15, 0.0)]
            elif l_side:
                bumper_msg = "LEFT"
                hazards = [(0.10, 0.18, 0.0), (0.0, 0.18, 0.0)]
            elif r_side:
                bumper_msg = "RIGHT"
                hazards = [(0.10, -0.18, 0.0), (0.0, -0.18, 0.0)]
            
            if bumper_msg:
                self.bumper_pub.publish(String(data=bumper_msg))
                self.publish_hazard_cloud(hazards)

            # --- Logica Bottone (Debounce) ---
            btn = int(sensors.get('BTN_START', 0))
            if btn == 1 and self.last_button_state == 0:
                self.button_pub.publish(Bool(data=True))
            self.last_button_state = btn

            # Carpet Detection
            brush_ma = sensors.get('BrushMotorInmA', sensors.get('BrushCurrent', 0))
            floor_msg = String()
            floor_msg.data = "carpet" if brush_ma > self.carpet_threshold else "hard"
            self.floor_pub.publish(floor_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Errore lettura sensori: {e}')

    def trigger_safety_reflex(self):
        now = time.time()
        if now < self.emergency_until: return

        self.get_logger().warn("SAFETY REFLEX! Drop/Mag detected.")
        self.emergency_until = now + 2.0
        with self.write_lock:
            self.ser.write(b"SetMotor LWheelDist -100 RWheelDist -100 Speed 100\n")
        self.publish_hazard_cloud()
        
        diag = DiagnosticStatus()
        diag.level = DiagnosticStatus.ERROR
        diag.name = "Safety System"
        diag.message = "Robot Stuck/Cliff Detected"
        self.diag_pub.publish(diag)

    def publish_hazard_cloud(self, points_list=None):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.height = 1
        msg.width = 10
        msg.is_dense = True
        msg.is_bigendian = False
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        points = []
        
        if points_list:
            for x, y, z in points_list:
                points.append(struct.pack('<fff', x, y, z))
        else:
            for i in range(10):
                y = -0.25 + (i * 0.05)
                points.append(struct.pack('<fff', 0.2, y, 0.0))
        
        msg.width = len(points)
        msg.row_step = 12 * msg.width
        msg.point_step = 12
        msg.data = b''.join(points)
        self.hazard_pub.publish(msg)

    def read_serial_loop(self):
        """Thread dedicato alla lettura continua della seriale."""
        while rclpy.ok():
            if not self.connected or self.ser is None:
                time.sleep(0.1)
                continue
            
            try:
                line = self.ser.readline()
                if not line: continue
                
                line = line.decode('utf-8', errors='ignore').strip()
                if not line: continue

                # 1. LIDAR Header Detection
                if line.startswith("AngleInDegree"):
                    self.current_scan_data = []
                    continue

                parts = line.split(',')
                
                # 2. LIDAR Data (4 colonne: Angle, Dist, Intensity, Error)
                if len(parts) == 4 and parts[0].isdigit():
                    try:
                        angle = int(parts[0])
                        dist_mm = int(parts[1])
                        intensity = int(parts[2])
                        error = int(parts[3])
                        
                        # Accumula
                        self.current_scan_data.append((angle, dist_mm / 1000.0, intensity))
                        
                        # Fine pacchetto (circa 360 gradi)
                        if angle >= 359:
                            self.publish_scan(self.current_scan_data)
                            self.current_scan_data = []
                            self.last_valid_packet_time = time.time()
                    except ValueError:
                        pass
                
                # 3. Sensor Data (CSV: Name,Value)
                elif len(parts) == 2:
                    try:
                        name = parts[0]
                        val = float(parts[1])
                        self.latest_sensors[name] = val
                        self.last_valid_packet_time = time.time()
                    except ValueError:
                        pass
                        
            except Exception as e:
                time.sleep(0.1)

    def lidar_loop(self):
        while rclpy.ok():
            if self.connected:
                self.send_command("GetLDSScan")
            time.sleep(1.0)

    def publish_scan(self, data):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_link'
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / 360.0
        msg.range_min = 0.02
        msg.range_max = self.max_range
        msg.ranges = [float('inf')] * 360
        msg.intensities = [0.0] * 360
        for angle, dist, intensity in data:
            if 0 <= angle < 360:
                msg.ranges[angle] = dist
                msg.intensities[angle] = float(intensity)
        self.scan_pub.publish(msg)

    def cmd_vel_callback(self, msg):
        if not self.connected or time.time() < self.emergency_until: return
        v, w = msg.linear.x, msg.angular.z
        vl_mm = int((v - (w * self.wheelbase / 2.0)) * 1000)
        vr_mm = int((v + (w * self.wheelbase / 2.0)) * 1000)
        speed = max(abs(vl_mm), abs(vr_mm))
        cmd = "SetMotor LWheelDist 0 RWheelDist 0 Speed 0" if speed == 0 else f"SetMotor LWheelDist {vl_mm} RWheelDist {vr_mm} Speed {speed}"
        self.send_command(cmd)

    def play_sound_callback(self, msg):
        if self.connected: self.send_command(f"PlaySound SoundID {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = NeatoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.write(b"SetLDSRotation Off\n")
            node.ser.write(b"SetMotor LWheelDist 0 RWheelDist 0 Speed 0\n")
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()