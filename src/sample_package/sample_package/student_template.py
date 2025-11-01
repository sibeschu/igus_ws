#!/usr/bin/env python3

"""
═══════════════════════════════════════════════════════════════════════════
    IGUS REBEL ROBOTER TEMPLATE FÜR STUDENTEN
    Hochschule Karlsruhe - Einführung Robotik
═══════════════════════════════════════════════════════════════════════════

VERWENDUNG:
    1. Starte den Roboter (Terminal 1):
       ros2 launch igus_rebel rebel.launch.py
    
    2. Starte MoveIt (Terminal 2):
       ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py use_gui:=true
    
    3. Starte dieses Template (Terminal 3):
       ros2 run sample_package student_template
    
    Oder: ros2 run sample_package student_template --loop (für Wiederholung)

VERFÜGBARE FUNKTIONEN:
    - move_to_pose(x, y, z, roll, pitch, yaw) : Bewegt Roboter zu Position
    - move_to_home()                           : Fährt zur sicheren Startposition
    - gripper_open()                           : Öffnet Greifer (Platzhalter)
    - gripper_close()                          : Schließt Greifer (Platzhalter)
    - wait(sekunden)                           : Wartet die angegebene Zeit

KOORDINATENSYSTEM:
    - X-Achse: Nach vorne (vom Roboter weg)
    - Y-Achse: Nach links
    - Z-Achse: Nach oben
    - Einheiten: Meter und Radiant
    - pi aus math importiert (z.B. pi/2 = 90°)

═══════════════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from scipy.spatial.transform import Rotation
from math import pi
import time
import sys


class StudentRobot(Node):
    """
    Einfache Roboter-Interface-Klasse für Studenten
    """
    
    def __init__(self):
        super().__init__('student_robot')
        
        # Action Client für MoveGroup
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Warte auf MoveGroup Server
        self.get_logger().info("⏳ Warte auf MoveGroup Server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ MoveGroup Server nicht erreichbar!")
            self.get_logger().error("   Bitte starte: ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py")
            raise RuntimeError("MoveGroup nicht verfügbar")
        
        self.get_logger().info("✓ Verbindung zu MoveGroup erfolgreich!")
        
        # Konfiguration
        self.planning_group = "igus_rebel_arm"
        self.end_effector = "tool0"
        self.max_retries = 3
        
    def _euler_to_quaternion(self, roll, pitch, yaw):
        """Konvertiert Euler-Winkel (roll, pitch, yaw) in Quaternion"""
        r = Rotation.from_euler('xyz', [roll, pitch, yaw])
        return r.as_quat()  # [x, y, z, w]
    
    def _send_goal_and_wait(self, goal_msg, action_name="Bewegung"):
        """
        Sendet Goal an MoveGroup und wartet auf Ergebnis
        Versucht mehrfach bei Fehler
        """
        for attempt in range(1, self.max_retries + 1):
            self.get_logger().info(f"🎯 {action_name} - Versuch {attempt}/{self.max_retries}")
            
            # Sende Goal
            future = self._action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if not future.result():
                self.get_logger().warning(f"⚠️  Goal wurde nicht akzeptiert (Versuch {attempt})")
                if attempt < self.max_retries:
                    time.sleep(1.0)
                    continue
                return False
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning(f"⚠️  Goal abgelehnt (Versuch {attempt})")
                if attempt < self.max_retries:
                    time.sleep(1.0)
                    continue
                return False
            
            # Warte auf Ergebnis
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
            
            result = result_future.result().result
            
            if result.error_code.val == 1:  # SUCCESS
                self.get_logger().info(f"✓ {action_name} erfolgreich!")
                return True
            else:
                self.get_logger().warning(
                    f"⚠️  {action_name} fehlgeschlagen (Error Code: {result.error_code.val}, Versuch {attempt})"
                )
                if attempt < self.max_retries:
                    time.sleep(2.0)
                    continue
                    
        self.get_logger().error(f"❌ {action_name} nach {self.max_retries} Versuchen fehlgeschlagen!")
        return False
    
    def move_to_home(self):
        """
        Fährt zur sicheren Home-Position
        
        Returns:
            bool: True bei Erfolg, False bei Fehler
        """
        self.get_logger().info("🏠 Fahre zur Home-Position...")
        
        # Sichere Home-Position (vermeidet Singularitäten)
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_values = [0.0, -pi/6, pi/3, 0.0, pi/6, 0.0]
        
        # Erstelle MoveGroup Goal
        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        
        # Setze Gelenkziele
        constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        
        return self._send_goal_and_wait(goal, "Home-Position")
    
    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        """
        Bewegt den Roboter zu einer bestimmten Pose
        
        Args:
            x, y, z (float): Position in Metern
            roll, pitch, yaw (float): Orientierung in Radiant
        
        Returns:
            bool: True bei Erfolg, False bei Fehler
        
        Beispiel:
            robot.move_to_pose(0.3, 0.0, 0.4, 0.0, pi/2, 0.0)
        """
        self.get_logger().info(f"📍 Bewege zu: x={x:.3f}m, y={y:.3f}m, z={z:.3f}m")
        self.get_logger().info(f"   Orientierung: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
        
        # Erstelle Ziel-Pose
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        
        q = self._euler_to_quaternion(roll, pitch, yaw)
        target.pose.orientation.x = q[0]
        target.pose.orientation.y = q[1]
        target.pose.orientation.z = q[2]
        target.pose.orientation.w = q[3]
        
        # Erstelle MoveGroup Goal
        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        
        # Position Constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = target.header
        pos_constraint.link_name = self.end_effector
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]
        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(target.pose)
        pos_constraint.weight = 1.0
        
        # Orientation Constraint
        ori_constraint = OrientationConstraint()
        ori_constraint.header = target.header
        ori_constraint.link_name = self.end_effector
        ori_constraint.orientation = target.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.01
        ori_constraint.absolute_y_axis_tolerance = 0.01
        ori_constraint.absolute_z_axis_tolerance = 0.01
        ori_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        
        return self._send_goal_and_wait(goal, f"Position ({x:.2f}, {y:.2f}, {z:.2f})")
    
    def gripper_open(self):
        """
        Öffnet den Greifer
        
        HINWEIS: Dies ist ein Platzhalter. Implementierung hängt vom Greifer ab.
        """
        self.get_logger().info("🤏 Greifer öffnen...")
        time.sleep(0.5)
        return True
    
    def gripper_close(self):
        """
        Schließt den Greifer
        
        HINWEIS: Dies ist ein Platzhalter. Implementierung hängt vom Greifer ab.
        """
        self.get_logger().info("🤏 Greifer schließen...")
        time.sleep(0.5)
        return True
    
    def wait(self, seconds):
        """
        Wartet die angegebene Zeit
        
        Args:
            seconds (float): Wartezeit in Sekunden
        """
        self.get_logger().info(f"⏱️  Warte {seconds} Sekunden...")
        time.sleep(seconds)


def student_program(robot):
    """
    ═══════════════════════════════════════════════════════════════════════
    STUDENT CODE BEREICH - Hier dein Programm schreiben!
    ═══════════════════════════════════════════════════════════════════════
    
    Verfügbare Funktionen:
    - robot.move_to_pose(x, y, z, roll, pitch, yaw)
    - robot.move_to_home()
    - robot.gripper_open()
    - robot.gripper_close()
    - robot.wait(sekunden)
    
    Beispiele:
    
    # Einfache Bewegung
    robot.move_to_pose(0.3, 0.0, 0.4, 0.0, pi/2, 0.0)
    robot.wait(1.0)
    
    # Pick and Place
    robot.move_to_pose(0.3, 0.1, 0.3, 0.0, pi/2, 0.0)  # Über Objekt
    robot.gripper_open()
    robot.move_to_pose(0.3, 0.1, 0.2, 0.0, pi/2, 0.0)  # Absenken
    robot.gripper_close()
    robot.move_to_pose(0.3, 0.1, 0.4, 0.0, pi/2, 0.0)  # Anheben
    robot.move_to_pose(0.3, -0.2, 0.3, 0.0, pi/2, 0.0) # Zu Ziel
    robot.gripper_open()
    
    # Mehrere Positionen
    positionen = [
        (0.3, 0.1, 0.4, 0.0, pi/2, 0.0),
        (0.3, -0.1, 0.4, 0.0, pi/2, 0.0),
        (0.2, 0.0, 0.5, 0.0, pi/2, 0.0)
    ]
    for x, y, z, roll, pitch, yaw in positionen:
        robot.move_to_pose(x, y, z, roll, pitch, yaw)
        robot.wait(0.5)
    
    ═══════════════════════════════════════════════════════════════════════
    """
    
    # ▼▼▼ DEIN CODE HIER ▼▼▼
    
    # Beispiel: Einfache Testbewegung
    robot.move_to_pose(0.3, 0.0, 0.4, 0.0, pi/2, 0.0)
    robot.wait(1.0)
    
    # ▲▲▲ DEIN CODE ENDE ▲▲▲


def main():
    """
    Hauptprogramm - Initialisiert Roboter und führt Student-Programm aus
    """
    # Prüfe Command-Line-Argumente
    loop_mode = '--loop' in sys.argv
    
    print("\n" + "═" * 70)
    print("  IGUS REBEL ROBOTER - STUDENT TEMPLATE")
    print("  Hochschule Karlsruhe")
    print("═" * 70)
    if loop_mode:
        print("  Modus: LOOP (Programm wird wiederholt)")
    else:
        print("  Modus: EINMALIG (Programm läuft einmal)")
    print("═" * 70 + "\n")
    
    rclpy.init()
    
    try:
        # Erstelle Roboter-Interface
        robot = StudentRobot()
        time.sleep(1.0)
        
        # Fahre zuerst zu Home
        print("\n" + "─" * 70)
        print("  INITIALISIERUNG: Fahre zu Home-Position")
        print("─" * 70)
        robot.move_to_home()
        robot.wait(1.0)
        
        # Loop oder Single-Run
        run_count = 0
        while True:
            run_count += 1
            
            print("\n" + "═" * 70)
            print(f"  STUDENT PROGRAMM AUSFÜHRUNG #{run_count}")
            print("═" * 70 + "\n")
            
            try:
                student_program(robot)
            except Exception as e:
                robot.get_logger().error(f"❌ Fehler im Student-Programm: {str(e)}")
                import traceback
                traceback.print_exc()
            
            # Zurück zu Home
            print("\n" + "─" * 70)
            print("  Fahre zurück zu Home-Position")
            print("─" * 70)
            robot.move_to_home()
            
            if not loop_mode:
                break
            
            print("\n⏸️  Warte 3 Sekunden vor nächstem Durchlauf...")
            time.sleep(3.0)
        
        print("\n" + "═" * 70)
        print("  ✓ PROGRAMM BEENDET")
        print("═" * 70 + "\n")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Programm durch Benutzer gestoppt (Ctrl+C)\n")
    except Exception as e:
        print(f"\n❌ Fehler: {str(e)}\n")
        import traceback
        traceback.print_exc()
    finally:
        try:
            robot.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
