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
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_param_builder import load_yaml

# MoveItPy Imports
from moveit.planning import MoveItPy

class RoboterTemplate(Node):
    def __init__(self):
        """
        Initialisiert den Roboter und alle notwendigen Verbindungen
        
        WICHTIG: Vor dem Ausführen dieses Templates muss move_group gestartet sein:
        Terminal 1: ros2 launch igus_rebel_moveit_config move_group.launch.py
        Terminal 2: ros2 run sample_package roboter_template
        """
        super().__init__('roboter_template')
        
        try:
            # MoveIt Initialisierung
            self.get_logger().info("Initialisiere MoveIt...")
            self.get_logger().info("Lade MoveIt Konfigurationsdateien...")
            
            # Lade alle benötigten Konfigurationsdateien
            moveit_config_pkg = get_package_share_directory('igus_rebel_moveit_config')
            description_pkg = get_package_share_directory('igus_rebel_description')
            
            # Lade URDF
            import subprocess
            urdf_file = os.path.join(description_pkg, 'urdf', 'igus_rebel_robot2.urdf.xacro')
            robot_description = subprocess.check_output([
                'xacro', urdf_file, 'hardware_protocol:=rebel'
            ]).decode('utf-8')
            
            # Lade SRDF
            srdf_file = os.path.join(moveit_config_pkg, 'config', 'igus_rebel2.srdf')
            with open(srdf_file, 'r') as f:
                robot_description_semantic = f.read()
            
            # Lade andere Konfigurationen
            kinematics_file = os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml')
            kinematics_config = load_yaml(Path(kinematics_file))
            
            ompl_file = os.path.join(moveit_config_pkg, 'config', 'ompl_planning.yaml')
            ompl_config = load_yaml(Path(ompl_file))
            
            joint_limits_file = os.path.join(moveit_config_pkg, 'config', 'joint_limits.yaml')
            joint_limits_config = load_yaml(Path(joint_limits_file))
            
            # Erstelle MoveItPy Konfiguration mit korrekter Struktur
            # Die planning pipeline Konfiguration muss im richtigen Format sein
            config_dict = {
                "robot_description": robot_description,
                "robot_description_semantic": robot_description_semantic,
                "robot_description_kinematics": kinematics_config,
                "robot_description_planning": joint_limits_config,
                "planning_pipelines": {
                    "pipeline_names": ["ompl"],
                    "default_pipeline": "ompl"
                },
                "ompl": {
                    "planning_plugin": "ompl_interface/OMPLPlanner",
                    "request_adapters": """default_planning_request_adapters/ResolveConstraintFrames \
                                          default_planning_request_adapters/ValidateWorkspaceBounds \
                                          default_planning_request_adapters/CheckStartStateBounds \
                                          default_planning_request_adapters/CheckStartStateCollision""",
                    "response_adapters": """default_planning_response_adapters/AddTimeOptimalParameterization \
                                           default_planning_response_adapters/ValidateSolution \
                                           default_planning_response_adapters/DisplayMotionPath""",
                    "start_state_max_bounds_error": 0.1,
                },
                "planning_scene_monitor_options": {
                    "name": "planning_scene_monitor",
                    "robot_description": "robot_description",
                    "joint_state_topic": "/joint_states",
                    "attached_collision_object_topic": "/attached_collision_object",
                    "publish_planning_scene_topic": "/publish_planning_scene",
                    "monitored_planning_scene_topic": "/monitored_planning_scene",
                    "wait_for_initial_state_timeout": 10.0,
                },
            }
            
            # Füge Planner-Konfigurationen hinzu
            if "planner_configs" in ompl_config:
                config_dict["ompl"]["planner_configs"] = ompl_config["planner_configs"]
            
            # Füge Arm-Gruppe Konfiguration hinzu
            if "igus_rebel_arm" in ompl_config:
                config_dict["ompl"]["igus_rebel_arm"] = ompl_config["igus_rebel_arm"]
            
            self.get_logger().info("Initialisiere MoveItPy mit Konfiguration...")
            self.moveit = MoveItPy(node_name="roboter_template", config_dict=config_dict)
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
            time.sleep(1.0)
            
            # Überprüfe ob Planning Component verfügbar ist
            if self.arm is None:
                self.get_logger().error("Planning Component nicht verfügbar")
                return False
            
            # Überprüfe Robot Model
            if self.robot_model is None:
                self.get_logger().error("Robot Model nicht verfügbar")
                return False
                
            return True
            
        except Exception as e:
            self.get_logger().error(f"Fehler bei der Roboterinitialisierung: {str(e)}")
            return False
    
    def _euler_to_quaternion(self, roll, pitch, yaw):
        """
        Konvertiert Euler-Winkel in Quaternion
        
        Args:
            roll, pitch, yaw (float): Winkel in Radiant
            
        Returns:
            tuple: (x, y, z, w) Quaternion
        """
        r = Rotation.from_euler('xyz', [roll, pitch, yaw])
        quat = r.as_quat()  # Returns [x, y, z, w]
        return quat

    def move_to_home(self):
        """
        Fährt den Roboter in eine sichere Ausgangsposition
        Diese Position vermeidet Singularitäten
        
        Returns:
            bool: True bei Erfolg, False bei Fehler
        """
        try:
            self.get_logger().info("Fahre zu Home-Position...")
            
            # Definiere eine sichere Home-Position (Winkel in Radiant)
            # Diese Werte vermeiden die singulare Position (alle Gelenke bei 0)
            joint_state = {
                'joint1': 0.0,
                'joint2': -pi/6,  # -30 Grad
                'joint3': pi/3,   # 60 Grad
                'joint4': 0.0,
                'joint5': pi/6,   # 30 Grad
                'joint6': 0.0
            }
            
            # Setze Start-State auf aktuellen Zustand
            self.arm.set_start_state_to_current_state()
            
            # Setze Ziel als Gelenkwerte
            self.arm.set_goal_state(joint_state_goal=joint_state)
            
            # Plane Bewegung
            plan_result = self.arm.plan()
            
            if plan_result:
                self.get_logger().info("Planung erfolgreich, führe Bewegung aus...")
                
                # Führe Bewegung aus
                self.moveit.execute(plan_result.trajectory, controllers=[])
                
                self.get_logger().info("Home-Position erreicht")
                return True
            else:
                self.get_logger().error("Planung zur Home-Position fehlgeschlagen")
                return False
            
        except Exception as e:
            self.get_logger().error(f"Fehler bei Home-Bewegung: {str(e)}")
            return False

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        """
        Bewegt den Roboter zu einer bestimmten Position und Orientierung

        Args:
            x, y, z (float): Zielposition in Metern (relativ zur Roboterbasis)
            roll, pitch, yaw (float): Zielorientierung in Radiant
                - roll: Rotation um X-Achse
                - pitch: Rotation um Y-Achse  
                - yaw: Rotation um Z-Achse

        Returns:
            bool: True bei erfolgreicher Bewegung, False bei Fehler
            
        Beispiel:
            # Bewege zu Position x=0.3m, y=0.0m, z=0.5m
            # mit Orientierung roll=0, pitch=90°, yaw=0
            roboter.move_to_pose(0.3, 0.0, 0.5, 0.0, pi/2, 0.0)
        """
        try:
            self.get_logger().info(f"Plane Bewegung zu Position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            self.get_logger().info(f"Orientierung: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")
            
            # Erstelle PoseStamped Nachricht
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "base_link"  # Referenzframe
            pose_goal.pose.position.x = x
            pose_goal.pose.position.y = y
            pose_goal.pose.position.z = z
            
            # Konvertiere Euler-Winkel in Quaternion
            q = self._euler_to_quaternion(roll, pitch, yaw)
            pose_goal.pose.orientation.x = q[0]
            pose_goal.pose.orientation.y = q[1]
            pose_goal.pose.orientation.z = q[2]
            pose_goal.pose.orientation.w = q[3]
            
            # Setze Start-State auf aktuellen Zustand
            self.arm.set_start_state_to_current_state()
            
            # Setze Ziel-Pose für den End-Effektor
            self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool0")
            
            # Plane Bewegung
            plan_result = self.arm.plan()
            
            if plan_result:
                self.get_logger().info("Planung erfolgreich, führe Bewegung aus...")
                
                # Führe Bewegung aus
                self.moveit.execute(plan_result.trajectory, controllers=[])
                
                self.get_logger().info("Zielposition erreicht")
                return True
            else:
                self.get_logger().error("Bewegungsplanung fehlgeschlagen - Position möglicherweise nicht erreichbar")
                return False
            
        except Exception as e:
            self.get_logger().error(f"Fehler bei Bewegung: {str(e)}")
            return False

    def gripper_open(self):
        """
        Öffnet den Greifer
        
        Returns:
            bool: True bei Erfolg
            
        Hinweis: Diese Funktion muss noch an den spezifischen Greifer angepasst werden
        """
        try:
            self.get_logger().info("Öffne Greifer...")
            
            # TODO: Implementierung für den Igus Greifer
            # Beispiel:
            # gripper_group = self.moveit.get_planning_component("gripper")
            # gripper_group.set_start_state_to_current_state()
            # gripper_group.set_goal_state(joint_state_goal={'gripper_joint': 0.04})
            # plan = gripper_group.plan()
            # if plan:
            #     self.moveit.execute(plan.trajectory, controllers=[])
            
            time.sleep(0.5)
            self.get_logger().info("Greifer geöffnet")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Fehler beim Öffnen des Greifers: {str(e)}")
            return False

    def gripper_close(self):
        """
        Schließt den Greifer
        
        Returns:
            bool: True bei Erfolg
            
        Hinweis: Diese Funktion muss noch an den spezifischen Greifer angepasst werden
        """
        try:
            self.get_logger().info("Schließe Greifer...")
            
            # TODO: Implementierung für den Igus Greifer
            # Beispiel:
            # gripper_group = self.moveit.get_planning_component("gripper")
            # gripper_group.set_start_state_to_current_state()
            # gripper_group.set_goal_state(joint_state_goal={'gripper_joint': 0.0})
            # plan = gripper_group.plan()
            # if plan:
            #     self.moveit.execute(plan.trajectory, controllers=[])
            
            time.sleep(0.5)
            self.get_logger().info("Greifer geschlossen")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Fehler beim Schließen des Greifers: {str(e)}")
            return False

    def wait(self, seconds):
        """
        Wartet eine bestimmte Zeit
        
        Args:
            seconds (float): Wartezeit in Sekunden
        """
        self.get_logger().info(f"Warte {seconds} Sekunden...")
        time.sleep(seconds)

def main():
    """
    Hauptprogramm - Hier können Studenten ihren Code schreiben
    """
    rclpy.init()
    
    try:
        # Initialisiere Roboter
        print("\n" + "=" * 60)
        print("Igus Rebel Roboter Template - MoveItPy")
        print("=" * 60)
        
        roboter = RoboterTemplate()
        
        # Warte bis der Roboter bereit ist
        time.sleep(2.0)
        
        # Fahre zuerst in Home-Position
        print("\n--- Fahre zu Home-Position ---")
        roboter.move_to_home()
        roboter.wait(1.0)
        
        print("\n" + "=" * 60)
        print("STUDENT CODE BEREICH - Schreibe deinen Code hier")
        print("=" * 60 + "\n")
        
        # ===============================================
        # HIER STUDENT CODE EINFÜGEN
        # ===============================================
        
        # Beispiele für Bewegungen:
        # 
        # 1. Einfache Bewegung zu einer Position:
        # roboter.move_to_pose(0.3, 0.0, 0.4, 0.0, pi/2, 0.0)
        # roboter.wait(1.0)
        #
        # 2. Pick and Place Beispiel:
        # print("Pick and Place Demonstration:")
        # roboter.move_to_pose(0.3, 0.1, 0.3, 0.0, pi/2, 0.0)  # Über Objekt
        # roboter.gripper_open()                                 # Greifer öffnen
        # roboter.move_to_pose(0.3, 0.1, 0.2, 0.0, pi/2, 0.0)  # Absenken
        # roboter.gripper_close()                                # Greifen
        # roboter.move_to_pose(0.3, 0.1, 0.4, 0.0, pi/2, 0.0)  # Anheben
        # roboter.move_to_pose(0.3, -0.2, 0.3, 0.0, pi/2, 0.0) # Zu Zielposition
        # roboter.gripper_open()                                 # Ablegen
        #
        # 3. Mehrere Positionen anfahren:
        # print("Fahre mehrere Positionen an:")
        # positionen = [
        #     (0.3, 0.1, 0.4, 0.0, pi/2, 0.0),
        #     (0.3, -0.1, 0.4, 0.0, pi/2, 0.0),
        #     (0.2, 0.0, 0.5, 0.0, pi/2, 0.0)
        # ]
        # for i, pos in enumerate(positionen):
        #     print(f"Position {i+1}/{len(positionen)}")
        #     roboter.move_to_pose(*pos)
        #     roboter.wait(0.5)
        #
        # 4. Kreisbewegung erstellen:
        # import math
        # print("Kreisbewegung:")
        # radius = 0.1
        # center_x, center_y, center_z = 0.3, 0.0, 0.4
        # for angle in range(0, 360, 30):  # Alle 30 Grad
        #     rad = math.radians(angle)
        #     x = center_x + radius * math.cos(rad)
        #     y = center_y + radius * math.sin(rad)
        #     roboter.move_to_pose(x, y, center_z, 0.0, pi/2, 0.0)
        #     roboter.wait(0.3)
        
        
        # ===============================================
        # ENDE STUDENT CODE
        # ===============================================
        
        print("\n" + "=" * 60)
        print("Programm beendet - Fahre zurück zu Home")
        print("=" * 60 + "\n")
        
        # Zurück zur Home-Position
        roboter.move_to_home()
        
    except KeyboardInterrupt:
        print("\nProgramm durch Benutzer beendet (Ctrl+C)")
    except Exception as e:
        print(f"\nFehler im Hauptprogramm: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        try:
            roboter.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("✓ ROS2 heruntergefahren")


if __name__ == '__main__':
    main()