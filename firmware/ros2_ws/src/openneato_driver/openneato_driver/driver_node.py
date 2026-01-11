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
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
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
        self.serial_lock = threading.Lock()
        self.connected = False
        self.emergency_until = 0.0
        self.last_valid_packet_time = time.time()

        # --- Publishers ---
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.hazard_pub = self.create_publisher(PointCloud2, '/hazard_cloud', 10)
        self.floor_pub = self.create_publisher(String, '/floor_type', 10)
        self.diag_pub = self.create_publisher(DiagnosticStatus, '/hazard_status', 10)
        # Publisher per sicurezza interna (Watchdog)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

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
        self.cleaning_sub = self.create_subscription(
            Bool,
            'cleaning/active',
            self.cleaning_callback,
            10
        )

        # --- Timers & Threads ---
        # Timer principale a 5Hz per housekeeping e batteria
        self.create_timer(0.2, self.main_loop)
        
        # Thread separato per il LiDAR (operazione bloccante/intensiva)
        self.lidar_thread = threading.Thread(target=self.lidar_loop, daemon=True)
        self.lidar_thread.start()

        self.get_logger().info(f'Neato Driver avviato su {self.serial_port} @ {self.baud_rate}')

    def connect_serial(self):
        """Gestisce la connessione alla porta seriale."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            
            self.ser = serial.Serial(
                self.serial_port, 
                self.baud_rate, 
                timeout=0.1
            )
            # Mettiamo il Neato in TestMode per ricevere comandi
            self.send_command("TestMode On")
            self.send_command("SetLDSRotation On")
            
            self.connected = True
            self.get_logger().info('Connessione seriale stabilita.')
            self.last_valid_packet_time = time.time()  # Reset watchdog
            return True
        except serial.SerialException as e:
            self.connected = False
            self.get_logger().error(f'Errore connessione seriale: {e}')
            return False

    def send_command(self, cmd):
        """Invia un comando seriale in modo thread-safe."""
        if not self.connected or not self.ser:
            return None
        
        response = []
        with self.serial_lock:
            try:
                self.ser.write(f"{cmd}\n".encode('utf-8'))
                # Legge l'echo del comando (opzionale, dipende dal firmware)
                # e la risposta
                time.sleep(0.01) # Breve pausa per dare tempo al buffer
            except serial.SerialException:
                self.connected = False
                return None
        return True

    def main_loop(self):
        """Loop a 5Hz: Gestione connessione e lettura batteria."""
        # --- Watchdog Check ---
        if self.connected and (time.time() - self.last_valid_packet_time > 1.0):
            self.get_logger().error("WATCHDOG: Serial Communication Lost (>1.0s)")
            self.handle_serial_failure()
            return

        if not self.connected:
            self.connect_serial()
            return

        # Lettura Sensori (Batteria, Drop, Mag, Brush)
        self.read_sensors()

    def handle_serial_failure(self):
        """Gestisce il timeout della seriale (Deadman Switch)."""
        # 1. Pubblica DiagnosticStatus ERROR
        diag = DiagnosticStatus()
        diag.level = DiagnosticStatus.ERROR
        diag.name = "Serial Connection"
        diag.message = "Serial Communication Lost"
        self.diag_pub.publish(diag)

        # 2. Pubblica velocità 0 su /cmd_vel (per sicurezza interna)
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        # 3. Tenta riconnessione
        self.connected = False
        # Tentativo di stop fisico se la porta è ancora mezza viva
        self.send_command("SetMotor 0 0 0")
        self.connect_serial()

    def read_sensors(self):
        """Invia GetAnalogSensors e parsa la risposta per batteria e sicurezza."""
        # Nota: La lettura richiede lock perché leggiamo più righe
        if not self.connected: 
            return

        try:
            with self.serial_lock:
                self.ser.reset_input_buffer()
                self.ser.write(b"GetAnalogSensors\n")
                
                # Dizionario per raccogliere tutti i sensori
                sensors = {}
                
                # Timeout di sicurezza per il loop di lettura
                start_time = time.time()
                while (time.time() - start_time) < 0.5:
                    raw_line = self.ser.readline()
                    try:
                        line = raw_line.decode('utf-8', errors='ignore').strip()
                    except ValueError:
                        continue

                    if not line:
                        continue
                    
                    # Validation: ASCII stampabile e lunghezza minima (es. "A,1")
                    if len(line) < 3 or not line.isprintable():
                        continue

                    parts = line.split(',')
                    if len(parts) >= 2:
                        try:
                            # Memorizza chiave e valore (convertito a float)
                            sensors[parts[0]] = float(parts[1])
                            # Pacchetto valido ricevuto
                            self.last_valid_packet_time = time.time()
                        except ValueError:
                            pass
                    
                    # Interrompiamo se abbiamo trovato un terminatore (spesso Ctrl-Z o riga vuota finale)
                    # Ma per semplicità qui leggiamo finché il buffer non è vuoto o timeout
                    if self.ser.in_waiting == 0:
                        break

                # --- 1. Pubblicazione Batteria ---
                if 'BatteryVoltageInmV' in sensors and 'BatteryLevel' in sensors:
                    msg = BatteryState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.voltage = sensors['BatteryVoltageInmV'] / 1000.0
                    msg.percentage = sensors['BatteryLevel'] / 100.0
                    msg.present = True
                    self.battery_pub.publish(msg)

                # --- 2. Safety Reflex (Drop & Mag) ---
                # DropSensor solitamente in mm (> 50mm = vuoto). MagSensor solitamente 0/1.
                drop_l = sensors.get('DropSensorLeft', 0)
                drop_r = sensors.get('DropSensorRight', 0)
                mag_l = sensors.get('MagSensorLeft', 0)
                mag_r = sensors.get('MagSensorRight', 0)

                # Soglie di attivazione
                is_hazard = (drop_l > 40 or drop_r > 40 or mag_l > 0 or mag_r > 0)

                if is_hazard:
                    self.trigger_safety_reflex()
                
                # --- 3. Carpet Detection ---
                # Cerca BrushCurrent o BrushMotorInmA
                brush_ma = sensors.get('BrushMotorInmA', sensors.get('BrushCurrent', 0))
                floor_msg = String()
                floor_msg.data = "carpet" if brush_ma > self.carpet_threshold else "hard"
                self.floor_pub.publish(floor_msg)

        except Exception as e:
            self.get_logger().warn(f'Errore lettura sensori: {e}')
            self.connected = False

    def trigger_safety_reflex(self):
        """Gestisce la reazione di emergenza a gradini o bande magnetiche."""
        now = time.time()
        # Se siamo già in emergenza, non fare nulla (stiamo già indietreggiando/aspettando)
        if now < self.emergency_until:
            return

        self.get_logger().warn("SAFETY REFLEX TRIGGERED! Drop/Mag detected.")
        self.emergency_until = now + 2.0

        # 1. Sovrascrivi movimento: Retromarcia rapida
        # SetMotor LDist RDist Speed -> -100mm a 100mm/s
        with self.serial_lock:
            self.ser.write(b"SetMotor -100 -100 100\n")

        # 2. Pubblica Hazard Cloud per Nav2
        self.publish_hazard_cloud()

        # 3. Pubblica stato diagnostico
        diag = DiagnosticStatus()
        diag.level = DiagnosticStatus.ERROR
        diag.name = "Safety System"
        diag.message = "Robot Stuck/Cliff Detected"
        self.diag_pub.publish(diag)

    def publish_hazard_cloud(self):
        """Crea un muro virtuale davanti al robot per Nav2."""
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        msg.height = 1
        msg.width = 10
        msg.is_dense = True
        msg.is_bigendian = False
        
        # Definizione campi (x, y, z float32)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        
        # Genera punti: Muro a x=0.2m, larghezza da -0.25 a 0.25
        points = []
        for i in range(10):
            y = -0.25 + (i * 0.05)
            points.append(struct.pack('<fff', 0.2, y, 0.0))
        
        msg.data = b''.join(points)
        self.hazard_pub.publish(msg)

    def lidar_loop(self):
        """Loop dedicato alla lettura del LiDAR."""
        while rclpy.ok():
            if not self.connected:
                time.sleep(1.0)
                continue

            try:
                scan_data = []
                with self.serial_lock:
                    self.ser.write(b"GetLDSScan\n")
                    # Il comando GetLDSScan restituisce un header e poi 360 righe
                    # Format: AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX
                    
                    # Leggi finché non trovi l'header o dati validi
                    header_found = False
                    start_time = time.time()
                    
                    # Leggiamo tutte le 360 righe
                    # Nota: Questo è un approccio semplificato. 
                    # Assicuriamo la pubblicazione di TUTTI i punti (High Res) senza decimazione.
                    # Un driver di produzione potrebbe usare un parser a stati.
                    lines_read = 0
                    while lines_read < 365 and (time.time() - start_time) < 1.0:
                        raw_line = self.ser.readline()
                        try:
                            line = raw_line.decode('utf-8', errors='ignore').strip()
                        except ValueError:
                            continue

                        if not line:
                            continue
                        
                        # Salta header o commenti
                        if line.startswith('Angle') or line.startswith('#') or line.startswith('GetLDSScan'):
                            continue

                        # Validation: ASCII stampabile e lunghezza minima
                        if len(line) < 5 or not line.isprintable():
                            continue

                        parts = line.split(',')
                        if len(parts) >= 4:
                            try:
                                angle = int(parts[0])
                                dist_mm = int(parts[1])
                                intensity = int(parts[2])
                                error = int(parts[3])
                                
                                # Filtra errori (0 = No Error)
                                if error == 0:
                                    scan_data.append((angle, dist_mm / 1000.0, intensity))
                                else:
                                    scan_data.append((angle, float('inf'), 0))
                                lines_read += 1
                                # Dati validi ricevuti
                                self.last_valid_packet_time = time.time()
                            except ValueError:
                                pass
                
                if len(scan_data) > 0:
                    self.publish_scan(scan_data)
                
                # Rate limiting del Lidar (circa 5Hz hardware nativo)
                time.sleep(0.15)

            except Exception as e:
                self.get_logger().warn(f'Errore Lidar Loop: {e}')
                time.sleep(1.0)

    def publish_scan(self, data):
        """Converte i dati raw in LaserScan."""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_link'
        
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / 360.0
        msg.range_min = 0.02
        msg.range_max = self.max_range
        
        # Inizializza array
        msg.ranges = [float('inf')] * 360
        msg.intensities = [0.0] * 360
        
        for angle, dist, intensity in data:
            if 0 <= angle < 360:
                msg.ranges[angle] = dist
                msg.intensities[angle] = float(intensity)
        
        self.scan_pub.publish(msg)

    def cmd_vel_callback(self, msg):
        """Callback per il movimento. Converte Twist in SetMotor."""
        if not self.connected:
            return

        # Safety Reflex: Ignora comandi se in stato di emergenza
        if time.time() < self.emergency_until:
            return

        v = msg.linear.x
        w = msg.angular.z

        # Cinematica Differenziale
        # v_left/right in m/s
        v_left = v - (w * self.wheelbase / 2.0)
        v_right = v + (w * self.wheelbase / 2.0)

        # Conversione in mm/s
        vl_mm = int(v_left * 1000)
        vr_mm = int(v_right * 1000)
        
        # Calcolo Speed (max delle due velocità assolute)
        speed = max(abs(vl_mm), abs(vr_mm))
        
        # Il comando SetMotor del Neato accetta: LWheelDist, RWheelDist, Speed
        # Per emulare un movimento continuo, diciamo al robot di muoversi per 
        # una distanza fittizia basata sulla velocità attuale per 1 secondo.
        # Se riceviamo cmd_vel a 10Hz, sovrascriveremo il comando prima che finisca.
        
        if speed == 0:
            cmd = "SetMotor 0 0 0"
        else:
            # Distanza da percorrere in 1 secondo a questa velocità
            dist_l = vl_mm 
            dist_r = vr_mm
            cmd = f"SetMotor {dist_l} {dist_r} {speed}"

        with self.serial_lock:
            try:
                self.ser.write(f"{cmd}\n".encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f'Errore invio motori: {e}')
                self.connected = False

    def play_sound_callback(self, msg):
        """Callback per riprodurre suoni di sistema."""
        if not self.connected:
            return
        # ID 0: Startup, 1: Start Cleaning, 2: Stop Cleaning, 3: Error
        self.send_command(f"PlaySound SoundID {msg.data}")

    def cleaning_callback(self, msg):
        """Attiva o disattiva i motori di pulizia (Vacuum, Main Brush, Side Brush)."""
        if not self.connected:
            return

        if msg.data:
            # Attivazione sequenziale motori
            self.send_command("SetMotor VacuumOn")
            self.send_command("SetMotor BrushOn")
            self.send_command("SetMotor SideBrushOn")
            self.get_logger().info("Cleaning Motors ENABLED")
        else:
            self.send_command("SetMotor VacuumOff")
            self.send_command("SetMotor BrushOff")
            self.send_command("SetMotor SideBrushOff")
            self.get_logger().info("Cleaning Motors DISABLED")

def main(args=None):
    rclpy.init(args=args)
    node = NeatoDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motori e laser alla chiusura
        if node.ser and node.ser.is_open:
            node.ser.write(b"SetLDSRotation Off\n")
            node.ser.write(b"SetMotor VacuumOff\n")
            node.ser.write(b"SetMotor BrushOff\n")
            node.ser.write(b"SetMotor SideBrushOff\n")
            node.ser.write(b"SetMotor 0 0 0\n")
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
