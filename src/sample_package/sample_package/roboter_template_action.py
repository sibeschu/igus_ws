#!/usr/bin/env python3

"""
Template für die Steuerung des Igus Rebel Roboters
Entwickelt für Studenten der Hochschule Karlsruhe

WICHTIG: Dieses Template benötigt einen laufenden move_group Server!

Starte zuerst in einem separaten Terminal:
    ros2 launch igus_rebel_moveit_config move_group.launch.py

Dann starte dieses Template:
    ros2 run sample_package roboter_template

HINWEIS ZU MOTOR TIMEOUTS:
Wenn der Roboter nach Verwendung des Keyboard-Controllers oder Servo in einen 
"Position Lag" Fehler gerät:
1. Stoppe dieses Programm (Ctrl+C)
2. Stoppe move_group (Ctrl+C im move_group Terminal)
3. Wenn mit echtem Roboter: Stoppe ros2 launch igus_rebel rebel.launch.py
4. Warte 3 Sekunden
5. Starte alles neu in umgekehrter Reihenfolge

Die Bewegungen sind sehr langsam (5% Geschwindigkeit) für maximale Stabilität.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from scipy.spatial.transform import Rotation
from math import pi
import time

class RoboterTemplate(Node):
    def __init__(self):
        """
        Initialisiert den Roboter und alle notwendigen Verbindungen
        """
        super().__init__('roboter_template')
        
        try:
            self.get_logger().info("Initialisiere Roboter-Interface...")
            
            # Action Client für MoveGroup
            self._move_group_action_client = ActionClient(
                self,
                MoveGroup,
                '/move_action'
            )
            
            # Warte auf Action Server
            self.get_logger().info("Warte auf move_group Action Server...")
            if not self._move_group_action_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("move_group Action Server nicht verfügbar!")
                self.get_logger().error("Bitte starte zuerst: ros2 launch igus_rebel_moveit_config move_group.launch.py")
                raise RuntimeError("move_group nicht verfügbar")
            
            # Planning Group Name
            self.planning_group = "igus_rebel_arm"
            self.end_effector_link = "tool0"
            
            self.get_logger().info("Roboter erfolgreich initialisiert!")
            self.get_logger().info(f"Planning Group: {self.planning_group}")
            
        except Exception as e:
            self.get_logger().error(f"Fehler bei der Initialisierung: {str(e)}")
            raise

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

    def _send_goal_and_wait(self, goal_msg):
        """
        Sendet ein Goal an move_group und wartet auf Ergebnis
        
        Args:
            goal_msg: MoveGroup.Goal Nachricht
            
        Returns:
            bool: True bei Erfolg, False bei Fehler
        """
        self.get_logger().info("Sende Bewegungsanfrage...")
        
        future = self._move_group_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal wurde abgelehnt!")
            return False
        
        self.get_logger().info("Goal akzeptiert, warte auf Ausführung...")
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        result = result_future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Bewegung erfolgreich ausgeführt!")
            return True
        else:
            self.get_logger().error(f"Bewegung fehlgeschlagen! Error code: {result.error_code.val}")
            return False

    def move_to_home(self):
        """
        Fährt den Roboter in eine sichere Ausgangsposition
        Diese Position vermeidet Singularitäten
        
        Returns:
            bool: True bei Erfolg, False bei Fehler
        """
        try:
            self.get_logger().info("Fahre zu Home-Position...")
            
            # Definiere sichere Home-Position als Gelenkwerte
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            joint_values = [0.0, -pi/6, pi/3, 0.0, pi/6, 0.0]
            
            # Erstelle Goal
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = self.planning_group
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = 5.0
            goal_msg.request.max_velocity_scaling_factor = 0.05  # Sehr langsam für Stabilität
            goal_msg.request.max_acceleration_scaling_factor = 0.05  # Sehr langsam für Stabilität
            
            # Setze Gelenkconstraints
            goal_msg.request.goal_constraints.append(Constraints())
            for name, value in zip(joint_names, joint_values):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = name
                joint_constraint.position = value
                joint_constraint.tolerance_above = 0.01
                joint_constraint.tolerance_below = 0.01
                joint_constraint.weight = 1.0
                goal_msg.request.goal_constraints[0].joint_constraints.append(joint_constraint)
            
            # Plane und führe aus
            goal_msg.planning_options.plan_only = False
            
            return self._send_goal_and_wait(goal_msg)
            
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
            
            # Erstelle Ziel-Pose
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.pose.position.x = x
            target_pose.pose.position.y = y
            target_pose.pose.position.z = z
            
            # Konvertiere Euler zu Quaternion
            q = self._euler_to_quaternion(roll, pitch, yaw)
            target_pose.pose.orientation.x = q[0]
            target_pose.pose.orientation.y = q[1]
            target_pose.pose.orientation.z = q[2]
            target_pose.pose.orientation.w = q[3]
            
            # Erstelle Goal
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = self.planning_group
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = 5.0
            goal_msg.request.max_velocity_scaling_factor = 0.05  # Sehr langsam für Stabilität
            goal_msg.request.max_acceleration_scaling_factor = 0.05  # Sehr langsam für Stabilität
            
            # Setze Pose Constraint
            from moveit_msgs.msg import PositionConstraint, OrientationConstraint
            from shape_msgs.msg import SolidPrimitive
            
            goal_constraint = Constraints()
            
            # Position Constraint
            position_constraint = PositionConstraint()
            position_constraint.header = target_pose.header
            position_constraint.link_name = self.end_effector_link
            position_constraint.target_point_offset.x = 0.0
            position_constraint.target_point_offset.y = 0.0
            position_constraint.target_point_offset.z = 0.0
            
            # Kleine Box um Zielposition
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [0.001, 0.001, 0.001]
            position_constraint.constraint_region.primitives.append(box)
            position_constraint.constraint_region.primitive_poses.append(target_pose.pose)
            position_constraint.weight = 1.0
            
            # Orientation Constraint
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header = target_pose.header
            orientation_constraint.link_name = self.end_effector_link
            orientation_constraint.orientation = target_pose.pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = 0.01
            orientation_constraint.absolute_y_axis_tolerance = 0.01
            orientation_constraint.absolute_z_axis_tolerance = 0.01
            orientation_constraint.weight = 1.0
            
            goal_constraint.position_constraints.append(position_constraint)
            goal_constraint.orientation_constraints.append(orientation_constraint)
            goal_msg.request.goal_constraints.append(goal_constraint)
            
            # Plane und führe aus
            goal_msg.planning_options.plan_only = False
            
            return self._send_goal_and_wait(goal_msg)
            
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
        self.get_logger().info("Öffne Greifer...")
        time.sleep(0.5)
        self.get_logger().info("Greifer geöffnet")
        return True

    def gripper_close(self):
        """
        Schließt den Greifer
        
        Returns:
            bool: True bei Erfolg
            
        Hinweis: Diese Funktion muss noch an den spezifischen Greifer angepasst werden
        """
        self.get_logger().info("Schließe Greifer...")
        time.sleep(0.5)
        self.get_logger().info("Greifer geschlossen")
        return True

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
        print("Igus Rebel Roboter Template - Action Client")
        print("Hochschule Karlsruhe")
        print("=" * 60)
        
        roboter = RoboterTemplate()
        
        # Warte kurz
        time.sleep(1.0)
        
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
        
        
        # ===============================================
        # ENDE STUDENT CODE
        # ===============================================
        
        print("\n" + "=" * 60)
        print("Programm beendet - Fahre zurück zu Home")
        print("=" * 60 + "\n")
        
        # Zurück zur Home-Position
        roboter.move_to_home()
        
    except KeyboardInterrupt:
        print("\n⚠️  Programm durch Benutzer beendet (Ctrl+C)")
    except Exception as e:
        print(f"\n❌ Fehler im Hauptprogramm: {str(e)}")
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
