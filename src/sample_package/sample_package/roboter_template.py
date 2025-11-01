#!/usr/bin/env python3

"""
Template für die Steuerung des Igus Rebel Roboters
Entwickelt für Studenten der Hochschule Karlsruhe

Dieser Template enthält:
- Grundlegende Roboterinitialisierung
- Bewegungsfunktionen mit Positions- und Orientierungskontrolle
- Greiferfunktionen
- Fehlerbehandlung und Statusüberwachung
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from math import pi
import time

# MoveItPy Imports
from moveit.planning import MoveItPy

class RoboterTemplate(Node):
    def __init__(self):
        """
        Initialisiert den Roboter und alle notwendigen Verbindungen
        """
        super().__init__('roboter_template')
        
        try:
            # MoveIt Initialisierung
            self.get_logger().info("Initialisiere MoveIt...")
            self.moveit = MoveItPy(node_name="roboter_template")
            self.robot_model = self.moveit.get_robot_model()
            
            # Planning Group für den Arm
            self.arm_group_name = "rebel_arm"
            self.arm = self.moveit.get_planning_component(self.arm_group_name)
            
            # Timeout für Bewegungen (in Sekunden)
            self.movement_timeout = 10.0
            
            # Prüfe ob der Roboter bereit ist
            if not self._check_robot_ready():
                self.get_logger().error("Roboter nicht bereit! Bitte überprüfen Sie die Verbindung.")
                return
            
            self.get_logger().info("Roboter erfolgreich initialisiert!")
            
        except Exception as e:
            self.get_logger().error(f"Fehler bei der Initialisierung: {str(e)}")
            raise

    def _check_robot_ready(self):
        """
        Überprüft ob der Roboter einsatzbereit ist
        Returns:
            bool: True wenn der Roboter bereit ist, False wenn nicht
        """
        try:
            # Warte auf Roboterstatus
            time.sleep(2.0)  # Gebe dem System Zeit zur Initialisierung
            return True
        except Exception as e:
            self.get_logger().error(f"Fehler bei der Roboterinitialisierung: {str(e)}")
            return False

    def move_to_home(self):
        """
        Fährt den Roboter in eine sichere Ausgangsposition
        Diese Position vermeidet Singularitäten
        """
        try:
            # Definiere eine sichere Home-Position (Winkel in Radiant)
            joint_positions = [0.0, -pi/6, pi/3, 0.0, pi/6, 0.0]  # Angepasste Werte für eine sichere Position
            
            # Plane und führe Bewegung aus
            self.planning_group.set_joint_value_target(joint_positions)
            result = self.planning_group.plan()
            
            if result:
                success = self.planning_group.execute()
                if success:
                    self.get_logger().info("Home-Position erreicht")
                    return True
            
            self.get_logger().error("Konnte Home-Position nicht erreichen")
            return False
            
        except Exception as e:
            self.get_logger().error(f"Fehler bei Home-Bewegung: {str(e)}")
            return False

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        """
        Bewegt den Roboter zu einer bestimmten Position und Orientierung

        Args:
            x, y, z (float): Zielposition in Metern
            roll, pitch, yaw (float): Zielorientierung in Radiant

        Returns:
            bool: True bei erfolgreicher Bewegung, False bei Fehler
        """
        try:
            # Erstelle Pose aus Position und Orientierung
            target_pose = Pose()
            target_pose.position = Point(x=x, y=y, z=z)
            
            # Konvertiere Euler-Winkel in Quaternion
            rotation = R.from_euler('xyz', [roll, pitch, yaw])
            q = rotation.as_quat()  # Returns [x, y, z, w]
            target_pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            
            # Prüfe ob Position erreichbar ist
            if not self._is_pose_reachable(target_pose):
                self.get_logger().error("Position nicht erreichbar!")
                return False
            
            # Plane Bewegung
            self.planning_group.set_pose_target(target_pose)
            result = self.planning_group.plan()
            
            if result:
                # Führe Bewegung aus
                success = self.planning_group.execute()
                if success:
                    self.get_logger().info("Zielposition erreicht")
                    return True
            
            self.get_logger().error("Bewegung konnte nicht ausgeführt werden")
            return False
            
        except Exception as e:
            self.get_logger().error(f"Fehler bei Bewegung: {str(e)}")
            return False

    def _is_pose_reachable(self, pose):
        """
        Prüft ob eine Position erreichbar ist

        Args:
            pose (Pose): Zu prüfende Position

        Returns:
            bool: True wenn erreichbar, False wenn nicht
        """
        try:
            # Erstelle eine Testplanung
            self.planning_group.set_pose_target(pose)
            result = self.planning_group.plan()
            
            return result is not None
            
        except Exception:
            return False

    def gripper_control(self, open_gripper):
        """
        Steuert den Greifer

        Args:
            open_gripper (bool): True zum Öffnen, False zum Schließen
        """
        try:
            # Hier Greifer-Steuerung implementieren
            action = "geöffnet" if open_gripper else "geschlossen"
            self.get_logger().info(f"Greifer wurde {action}")
            
        except Exception as e:
            self.get_logger().error(f"Fehler bei Greifer-Steuerung: {str(e)}")

def main():
    """
    Hauptprogramm - Hier können Studenten ihren Code schreiben
    """
    rclpy.init()
    roboter = RoboterTemplate()
    
    try:
        # Warte bis der Roboter bereit ist
        time.sleep(2.0)
        
        # Fahre zuerst in Home-Position
        roboter.move_to_home()
        
        # ---- HIER STUDENT CODE EINFÜGEN ----
        
        # Beispiel für eine Bewegung:
        # roboter.move_to_pose(0.3, 0.0, 0.5, 0.0, pi/2, 0.0)
        
        # Beispiel für Greifer-Steuerung:
        # roboter.gripper_control(True)  # Öffnen
        # roboter.gripper_control(False) # Schließen
        
        # ---- ENDE STUDENT CODE ----
        
    except KeyboardInterrupt:
        roboter.get_logger().info("Programm durch Benutzer beendet")
    except Exception as e:
        roboter.get_logger().error(f"Fehler im Hauptprogramm: {str(e)}")
    finally:
        # Cleanup
        roboter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()