#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════
    IGUS REBEL - Student Robot Control Template
    Hochschule Karlsruhe - Einführung Robotik
═══════════════════════════════════════════════════════════════════════════

VERWENDUNG:
    1. Starte den Roboter (Terminal 1):
       ros2 launch igus_rebel rebel.launch.py
    
    2. Starte MoveIt (Terminal 2):
       ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py use_gui:=true
    
    3. Starte dieses Programm (Terminal 3):
       ros2 run igus_student student_control

VERFÜGBARE FUNKTIONEN:
    - move_to_pose(x, y, z, roll, pitch, yaw) : Bewegt Roboter zu Position mit Orientierung
    - move_to_home()                           : Fährt zur Home-Position

KOORDINATEN:
    - x, y, z: Position in Metern (float)
    - roll, pitch, yaw: Orientierung in Radiant (float)
    
BEISPIEL ORIENTIERUNG:
    - Greifer nach unten: roll=0, pitch=pi/2, yaw=0
    - Greifer horizontal: roll=0, pitch=0, yaw=0
    - pi/2 = 90°, pi = 180°, -pi/2 = -90°

═══════════════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint, BoundingVolume,
    MotionPlanRequest, PlanningOptions
)
from scipy.spatial.transform import Rotation
from math import pi

# ═══════════════════════════════════════════════════════════════════════════
# KONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════

PLANNING_GROUP = "igus_rebel_arm"
ENDEFFECTOR_LINK = "link6"
PLANNING_FRAME = "world"
POSE_TOLERANCE_POSITION = 0.005      # 5 mm
POSE_TOLERANCE_ROTATION = 0.02       # ~1.1 Grad

# Home Position (sicher, mittig)
HOME_POSITION = (0.4, 0.0, 0.40)        # x, y, z in Metern
HOME_ORIENTATION = (pi, 0.0, 0.0)     # roll, pitch, yaw (Greifer nach unten)


# ═══════════════════════════════════════════════════════════════════════════
# ROBOT CONTROL CLASS (Nicht ändern!)
# ═══════════════════════════════════════════════════════════════════════════

class RobotController(Node):
    """Basis-Controller für MoveIt MoveGroup Actions"""
    
    def __init__(self):
        super().__init__("student_robot_controller")
        self.callback_group = ReentrantCallbackGroup()
        
        # Action Client für MoveGroup
        self.client = ActionClient(
            self, 
            MoveGroup, 
            "/move_action", 
            callback_group=self.callback_group
        )
        
        # Warte auf MoveGroup Server
        self.get_logger().info("⏳ Warte auf MoveGroup Server...")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ MoveGroup Server nicht erreichbar!")
            raise RuntimeError("MoveGroup Server nicht verfügbar")
        
        self.get_logger().info("✓ Verbunden mit MoveGroup!")
    
    def _euler_to_quaternion(self, roll, pitch, yaw):
        """Konvertiert Euler-Winkel (roll, pitch, yaw) zu Quaternion (w, x, y, z)"""
        r = Rotation.from_euler('xyz', [roll, pitch, yaw])
        q = r.as_quat()  # returns [x, y, z, w]
        return q[3], q[0], q[1], q[2]  # return as (w, x, y, z)
    
    def _make_pose_goal_constraints(self, pose_stamped: PoseStamped) -> Constraints:
        """Erstellt Position und Orientierungs-Constraints für MoveIt"""
        
        # Position Constraint (Kugel mit Toleranz)
        sphere = SolidPrimitive(
            type=SolidPrimitive.SPHERE, 
            dimensions=[POSE_TOLERANCE_POSITION]
        )
        bounding_volume = BoundingVolume(
            primitives=[sphere], 
            primitive_poses=[pose_stamped.pose]
        )
        
        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = ENDEFFECTOR_LINK
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        
        # Orientierung Constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.link_name = ENDEFFECTOR_LINK
        orientation_constraint.orientation = pose_stamped.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = POSE_TOLERANCE_ROTATION
        orientation_constraint.absolute_y_axis_tolerance = POSE_TOLERANCE_ROTATION
        orientation_constraint.absolute_z_axis_tolerance = POSE_TOLERANCE_ROTATION
        orientation_constraint.weight = 1.0
        
        return Constraints(
            position_constraints=[position_constraint],
            orientation_constraints=[orientation_constraint]
        )
    
    def send_pose_goal(self, x, y, z, roll, pitch, yaw):
        """
        Sendet Pose-Ziel an MoveGroup und führt Bewegung aus
        
        Args:
            x, y, z: Position in Metern
            roll, pitch, yaw: Orientierung in Radiant
        
        Returns:
            bool: True bei Erfolg, False bei Fehler
        """
        self.get_logger().info(f"🎯 Ziel: ({x:.3f}, {y:.3f}, {z:.3f})")
        self.get_logger().info(f"   Rotation: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
        
        # 1) Konvertiere Euler zu Quaternion
        qw, qx, qy, qz = self._euler_to_quaternion(roll, pitch, yaw)
        
        # 2) Erstelle Ziel-Pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = PLANNING_FRAME
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z
        goal_pose.pose.orientation.w = qw
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        
        # 3) Erstelle Constraints
        goal_constraints = self._make_pose_goal_constraints(goal_pose)
        
        # 4) Erstelle Motion Plan Request
        motion_request = MotionPlanRequest()
        motion_request.group_name = PLANNING_GROUP
        motion_request.goal_constraints = [goal_constraints]
        
        # 5) Planning Options (plan + execute)
        planning_options = PlanningOptions(plan_only=False)
        
        # 6) MoveGroup Goal
        move_group_goal = MoveGroup.Goal(
            request=motion_request,
            planning_options=planning_options
        )
        
        # 7) Sende Goal an MoveGroup
        self.get_logger().info("📤 Sende Goal an MoveGroup...")
        send_future = self.client.send_goal_async(move_group_goal)
        rclpy.spin_until_future_complete(self, send_future)
        
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal wurde abgelehnt!")
            return False
        
        self.get_logger().info("⏳ Warte auf Ausführung...")
        
        # 8) Warte auf Ergebnis
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        
        # SUCCESS = 1 (moveit_msgs/MoveItErrorCodes)
        success = (result.error_code.val == 1)
        
        if success:
            self.get_logger().info("✅ Bewegung erfolgreich!")
        else:
            self.get_logger().error(f"❌ Fehler! Error Code: {result.error_code.val}")
        
        return success


# ═══════════════════════════════════════════════════════════════════════════
# GLOBALE ROBOT INSTANZ
# ═══════════════════════════════════════════════════════════════════════════

_robot_instance = None


def _get_robot():
    """Gibt die Robot-Controller Instanz zurück"""
    global _robot_instance
    if _robot_instance is None:
        raise RuntimeError("Robot nicht initialisiert! main() zuerst ausführen.")
    return _robot_instance


# ═══════════════════════════════════════════════════════════════════════════
# STUDENT INTERFACE FUNKTIONEN
# ═══════════════════════════════════════════════════════════════════════════

def move_to_pose(x, y, z, roll, pitch, yaw):
    """
    Bewegt den Roboter zu einer Position mit Orientierung
    
    Args:
        x, y, z: Position in Metern (float)
        roll, pitch, yaw: Orientierung in Radiant (float)
    
    Returns:
        bool: True bei Erfolg, False bei Fehler
    
    Beispiel:
        move_to_pose(0.4, 0.0, 0.3, 0.0, pi/2, 0.0)
    """
    robot = _get_robot()
    return robot.send_pose_goal(x, y, z, roll, pitch, yaw)


def move_to_home():
    """
    Fährt zur Home-Position
    
    Returns:
        bool: True bei Erfolg, False bei Fehler
    """
    print("🏠 Fahre zur Home-Position...")
    x, y, z = HOME_POSITION
    roll, pitch, yaw = HOME_ORIENTATION
    return move_to_pose(x, y, z, roll, pitch, yaw)


# ═══════════════════════════════════════════════════════════════════════════
# STUDENT PROGRAMM BEREICH
# ═══════════════════════════════════════════════════════════════════════════

def student_program():
    """
    ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
    HIER DEIN PROGRAMM SCHREIBEN!
    ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
    
    Verfügbare Funktionen:
    - move_to_pose(x, y, z, roll, pitch, yaw)
    - move_to_home()
    
    Beispiele:
    
    # Beispiel 1: Zur Home-Position fahren
    move_to_home()
    
    # Beispiel 2: Zu einer Position fahren (Greifer nach unten)
    move_to_pose(0.3, 0.1, 0.4, 0.0, pi/2, 0.0)
    
    # Beispiel 3: Mehrere Positionen nacheinander
    move_to_pose(0.4, 0.0, 0.3, 0.0, pi/2, 0.0)
    move_to_pose(0.4, 0.1, 0.3, 0.0, pi/2, 0.0)
    move_to_pose(0.4, 0.0, 0.4, 0.0, pi/2, 0.0)
    
    """
    
    print("\n" + "═" * 70)
    print("  STUDENT PROGRAMM STARTET")
    print("═" * 70 + "\n")
    
    # ▼▼▼ DEIN CODE HIER ▼▼▼
    
    # Beispiel: Fahre zur Home-Position
    move_to_home()
    
    # Beispiel: Fahre zu einer Position (Greifer nach unten zeigend)
    move_to_pose(0.3, 0.0, 0.35, 0.0, pi/2, 0.0)
    
    # ▲▲▲ DEIN CODE ENDE ▲▲▲
    
    print("\n" + "═" * 70)
    print("  STUDENT PROGRAMM BEENDET")
    print("═" * 70 + "\n")


# ═══════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════

def main():
    """Hauptprogramm - Initialisiert Robot und führt Student-Programm aus"""
    global _robot_instance
    
    print("\n" + "═" * 70)
    print("  IGUS REBEL - STUDENT ROBOT CONTROL")
    print("  Hochschule Karlsruhe")
    print("═" * 70 + "\n")
    
    # ROS2 initialisieren
    rclpy.init()
    
    try:
        # Robot Controller erstellen
        _robot_instance = RobotController()
        
        # Student Programm ausführen
        student_program()
        
    except KeyboardInterrupt:
        print("\n⚠️  Programm durch Benutzer gestoppt (Ctrl+C)\n")
    except Exception as e:
        print(f"\n❌ Fehler: {str(e)}\n")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if _robot_instance is not None:
            _robot_instance.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
