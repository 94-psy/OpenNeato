#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, BatteryState

# Assumiamo che l'azione sia definita in openneato_interfaces
# Definizione ipotetica DockToBase.action:
# ---
# bool success
# ---
# string current_state
# float32 distance
try:
    from openneato_interfaces.action import DockToBase
except ImportError:
    # Fallback per l'analisi statica se il pacchetto non è compilato
    class DockToBase:
        class Goal: pass
        class Result:
            success = False
        class Feedback:
            current_state = ""
            distance = 0.0

class DockingServer(Node):
    def __init__(self):
        super().__init__('docking_server')
        
        self._action_server = ActionServer(
            self,
            DockToBase,
            'dock_to_base',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.sub_battery = self.create_subscription(BatteryState, 'battery_state', self.battery_callback, 10)

        self.latest_scan = None
        self.charging_current = 0.0
        self.is_docked = False
        
        # Parametri
        self.docking_speed = -0.02  # m/s (retromarcia lenta)
        self.approach_speed = 0.1
        self.intensity_threshold = 2000.0 # Valore ipotetico per riflettori
        self.charging_threshold = 0.5 # Ampere

        self.get_logger().info("Docking Server Ready")

    def goal_callback(self, goal_request):
        self.get_logger().info('Ricevuta richiesta di docking')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Richiesta di cancellazione docking')
        self.stop_robot()
        return CancelResponse.ACCEPT

    def scan_callback(self, msg):
        self.latest_scan = msg

    def battery_callback(self, msg):
        # Assumiamo che current sia positivo in carica o ci sia un flag
        # Se il driver riporta negativo in scarica, usiamo abs() o logica specifica
        self.charging_current = msg.current
        if self.charging_current > self.charging_threshold:
            self.is_docked = True

    def stop_robot(self):
        msg = Twist()
        self.pub_vel.publish(msg)

    async def execute_callback(self, goal_handle):
        feedback_msg = DockToBase.Feedback()
        result = DockToBase.Result()

        # Fase 1: Avvicinamento a (0,0) - Presunta base
        # Nota: In un sistema reale, questo userebbe Nav2 per pianificare un percorso.
        # Qui simuliamo l'avvicinamento o assumiamo di essere già in zona "pre-dock".
        feedback_msg.current_state = "APPROACHING_PRE_DOCK"
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info("Avvicinamento alla zona di pre-docking...")
        time.sleep(2.0) # Simulazione navigazione

        # Fase 2: Scansione Riflettori
        feedback_msg.current_state = "SCANNING_REFLECTORS"
        goal_handle.publish_feedback(feedback_msg)
        
        target_angle = self.find_dock_reflector()
        
        if target_angle is None:
            self.get_logger().error("Riflettori base non trovati!")
            result.success = False
            goal_handle.abort()
            return result

        # Fase 3: Allineamento (Rotazione verso i riflettori - 180 gradi per andare in retromarcia)
        # Il Neato ha i contatti dietro. Dobbiamo puntare il retro verso il target.
        # Lidar 0 è avanti. Target angle è relativo a 0.
        # Vogliamo che target_angle diventi 180 (pi greco).
        
        feedback_msg.current_state = "ALIGNING"
        goal_handle.publish_feedback(feedback_msg)
        self.align_robot(target_angle)

        # Fase 4: Retromarcia controllata
        feedback_msg.current_state = "DOCKING_REVERSE"
        goal_handle.publish_feedback(feedback_msg)
        
        self.is_docked = False
        timeout = 30.0 # secondi
        start_time = time.time()

        twist = Twist()
        twist.linear.x = self.docking_speed

        while rclpy.ok() and not self.is_docked:
            if time.time() - start_time > timeout:
                self.stop_robot()
                self.get_logger().error("Timeout docking: Nessuna corrente rilevata.")
                result.success = False
                goal_handle.abort()
                return result

            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                return result

            self.pub_vel.publish(twist)
            time.sleep(0.1)

        # Fase 5: Secure Contact (Wiggle & Push)
        # Non ci fermiamo subito. Spingiamo per garantire il contatto.
        feedback_msg.current_state = "SECURE_CONTACT"
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info("Contatto rilevato! Esecuzione Wiggle & Push...")

        secure_start = time.time()
        while (time.time() - secure_start) < 2.0:
            # Spinta costante indietro
            twist.linear.x = -0.04
            # Oscillazione (Wiggle)
            # angular.z = 0.3 * sin(time * 10)
            t_osc = time.time() - secure_start
            twist.angular.z = 0.3 * math.sin(t_osc * 10.0)
            
            self.pub_vel.publish(twist)
            time.sleep(0.05)
        
        # Stop Finale
        self.stop_robot()
        
        feedback_msg.current_state = "CHARGING"
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f"Docking completato. Corrente: {self.charging_current}A")
        result.success = True
        goal_handle.succeed()
        return result

    def find_dock_reflector(self):
        """Calcola il centroide ponderato dei riflettori (Weighted Average)."""
        if self.latest_scan is None:
            return None
        
        sum_sin = 0.0
        sum_cos = 0.0
        intensity_sum = 0.0
        
        for i, intensity in enumerate(self.latest_scan.intensities):
            if intensity > self.intensity_threshold:
                angle = self.latest_scan.angle_min + (i * self.latest_scan.angle_increment)
                sum_sin += intensity * math.sin(angle)
                sum_cos += intensity * math.cos(angle)
                intensity_sum += intensity
        
        if intensity_sum > 0:
            return math.atan2(sum_sin, sum_cos)
            
        return None

    def align_robot(self, target_angle_rad):
        """Ruota il robot finché il target non è a 180 gradi (PI) rispetto al fronte."""
        self.get_logger().info(f"Allineamento al target: {target_angle_rad:.2f} rad")
        
        kp = 1.5
        timeout = 10.0
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time < timeout):
            # Aggiorniamo la posizione del target (il robot sta ruotando)
            current_target = self.find_dock_reflector()
            
            if current_target is None:
                # Se perdiamo il target, usiamo l'ultimo noto o ci fermiamo
                # Qui ci fermiamo per sicurezza
                self.stop_robot()
                return

            # Errore: Vogliamo che current_target sia PI (3.14)
            error = current_target - math.pi
            
            # Normalizzazione angolo (-pi, pi)
            error = math.atan2(math.sin(error), math.cos(error))
            
            if abs(error) < 0.05: # Tolleranza ~3 gradi
                self.stop_robot()
                break
            
            twist = Twist()
            twist.angular.z = kp * error
            # Saturazione
            twist.angular.z = max(min(twist.angular.z, 1.0), -1.0)
            
            self.pub_vel.publish(twist)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = DockingServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
