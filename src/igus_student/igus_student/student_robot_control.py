#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════
    IGUS REBEL - Student Robot Control Template
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

COLLISION OBJECTS:
    - Definiert in STATIC_COLLISION_OBJECTS (siehe Konfiguration oben)
    - Werden beim Start automatisch geladen
    - Um neue Objekte hinzuzufügen, STATIC_COLLISION_OBJECTS bearbeiten

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
    MotionPlanRequest, PlanningOptions, CollisionObject, PlanningScene
)
from moveit_msgs.srv import ApplyPlanningScene
from scipy.spatial.transform import Rotation
from math import pi

# ═══════════════════════════════════════════════════════════════════════════
# KONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════

PLANNING_GROUP = "igus_rebel_arm"
ENDEFFECTOR_LINK = "link6"
PLANNING_FRAME = "world"
POSE_TOLERANCE_POSITION = 0.01      # 10 mm
POSE_TOLERANCE_ROTATION = 0.05       # ~3 Grad

# Home Position (sicher, mittig)
HOME_POSITION = (0.4, 0.0, 0.40)        # x, y, z in Metern
HOME_ORIENTATION = (pi, 0.0, 0.0)     # roll, pitch, yaw (Greifer nach unten)

# ═══════════════════════════════════════════════════════════════════════════
# STATISCHE COLLISION OBJECTS
# ═══════════════════════════════════════════════════════════════════════════

# Definiere hier deine festen Collision Objects
STATIC_COLLISION_OBJECTS = [
    # Beispiel Tisch
    {
        'name': 'table',
        'shape_type': 'box',
        'dimensions': [1.2, 0.8, 0.02],  # [width, depth, height] in Metern
        'position': (0.5, 0.0, -0.02),   # (x, y, z) - z so dass Oberseite bei z=0
        'orientation': (0.0, 0.0, 0.0, 1.0)  # (x, y, z, w) Quaternion - MUSS Float sein!
    },
    # Weitere Objekte hier hinzufügen:
    # {
    #     'name': 'wall',
    #     'shape_type': 'box', 
    #     'dimensions': [0.1, 2.0, 1.0],
    #     'position': (0.8, 0.0, 0.5),
    #     'orientation': (0.0, 0.0, 0.0, 1.0)
    # }
]


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
        
        # Service Client für Planning Scene
        self.planning_scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
            callback_group=self.callback_group
        )
        
        # Publisher für Planning Scene (alternative method)
        self.planning_scene_publisher = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )
        
        # Warte auf MoveGroup Server
        self.get_logger().info("Warte auf MoveGroup Server...")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup Server nicht erreichbar!")
            raise RuntimeError("MoveGroup Server nicht verfügbar")
        
        # Warte auf Planning Scene Service
        self.get_logger().info("Warte auf Planning Scene Service...")
        if not self.planning_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning("Planning Scene Service nicht verfügbar. Nutze Publisher stattdessen.")
        
        self.get_logger().info("✓ Verbunden mit MoveGroup!")
        
        # Statische Collision Objects hinzufügen
        self._add_static_collision_objects()
    
    def _add_static_collision_objects(self):
        """Fügt alle statischen Collision Objects beim Start hinzu"""
        try:
            if not STATIC_COLLISION_OBJECTS:
                self.get_logger().info("Keine statischen Collision Objects definiert")
                return
            
            planning_scene = PlanningScene()
            planning_scene.world.collision_objects = []
            
            for obj_config in STATIC_COLLISION_OBJECTS:
                # Collision Object erstellen
                collision_object = CollisionObject()
                collision_object.header.frame_id = PLANNING_FRAME
                collision_object.id = obj_config['name']
                
                # Shape definieren
                primitive = SolidPrimitive()
                shape_type = obj_config['shape_type']
                
                if shape_type == 'box':
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [float(d) for d in obj_config['dimensions']]  # [x, y, z]
                elif shape_type == 'sphere':
                    primitive.type = SolidPrimitive.SPHERE
                    primitive.dimensions = [float(d) for d in obj_config['dimensions']]  # [radius]
                elif shape_type == 'cylinder':
                    primitive.type = SolidPrimitive.CYLINDER
                    primitive.dimensions = [float(d) for d in obj_config['dimensions']]  # [height, radius]
                
                # Position und Orientierung setzen
                pose = PoseStamped()
                pose.header.frame_id = PLANNING_FRAME
                position = obj_config['position']
                orientation = obj_config['orientation']
                
                pose.pose.position.x = float(position[0])
                pose.pose.position.y = float(position[1])
                pose.pose.position.z = float(position[2])
                pose.pose.orientation.x = float(orientation[0])
                pose.pose.orientation.y = float(orientation[1])
                pose.pose.orientation.z = float(orientation[2])
                pose.pose.orientation.w = float(orientation[3])
                
                collision_object.primitives = [primitive]
                collision_object.primitive_poses = [pose.pose]
                collision_object.operation = CollisionObject.ADD
                
                planning_scene.world.collision_objects.append(collision_object)
                self.get_logger().info(f"✓ Statisches Collision Object '{obj_config['name']}' vorbereitet")
            
            # Alle Objekte gleichzeitig hinzufügen
            planning_scene.is_diff = True
            self.planning_scene_publisher.publish(planning_scene)
            
            # Kurz warten, damit die Nachricht verarbeitet wird
            import time
            time.sleep(0.5)  # Etwas länger warten für bessere Sichtbarkeit
            
            # Nochmals publizieren für bessere Zuverlässigkeit
            self.planning_scene_publisher.publish(planning_scene)
            time.sleep(0.1)
            
            self.get_logger().info(f"✓ {len(STATIC_COLLISION_OBJECTS)} statische Collision Objects hinzugefügt")
            self.get_logger().info("📋 Objects sollten in RViz unter 'PlanningScene' > 'Scene Geometry' sichtbar sein")
            
        except Exception as e:
            self.get_logger().error(f"Fehler beim Hinzufügen statischer Collision Objects: {e}")
    
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
        self.get_logger().info(f"Ziel: ({x:.3f}, {y:.3f}, {z:.3f})")
        self.get_logger().info(f"Rotation: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
        
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

        motion_request.max_velocity_scaling_factor = 0.3
        motion_request.max_acceleration_scaling_factor = 0.3
        
        # 5) Planning Options (plan + execute)
        planning_options = PlanningOptions(plan_only=False)
        
        # 6) MoveGroup Goal
        move_group_goal = MoveGroup.Goal(
            request=motion_request,
            planning_options=planning_options
        )
        
        # 7) Sende Goal an MoveGroup (async mit Spinning)
        try:
            self.get_logger().info("Sende Goal an MoveGroup...")
            send_goal_future = self.client.send_goal_async(move_group_goal)
            
            # Warte auf Goal Acceptance (mit Spinning für Nachrichtenverarbeitung)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                raise RuntimeError("Goal wurde vom Server abgelehnt!")
            
            self.get_logger().info("Goal akzeptiert, warte auf Ausführung...")
            
            # Warte auf Ergebnis (mit Spinning)
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()
            
            # SUCCESS = 1 (moveit_msgs/MoveItErrorCodes)
            if result.result.error_code.val != 1:
                raise RuntimeError(f"Bewegung fehlgeschlagen! Error Code: {result.result.error_code.val}")
            
            self.get_logger().info("✓ Bewegung erfolgreich!")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Fehler: {e}")
            raise
        #goal_handle = send_future.result()
        #if not goal_handle.accepted:
        #    self.get_logger().error("Goal wurde abgelehnt!")
        #    return False
        
        #self.get_logger().info("Warte auf Ausführung...")
        
        # 8) Warte auf Ergebnis
        # result_future = goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self, result_future)
        # result = result_future.result().result
        
        # SUCCESS = 1 (moveit_msgs/MoveItErrorCodes)
        #success = (result.error_code.val == 1)
        
        #if success:
        #    self.get_logger().info("Bewegung erfolgreich!")
        #else:
        #    self.get_logger().error(f"Fehler! Error Code: {result.error_code.val}")
        
        #return send_future.result()


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
    print("Fahre zur Home-Position...")
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
    
    Collision Objects:
    - Werden automatisch beim Start aus STATIC_COLLISION_OBJECTS geladen
    - Um neue Objekte hinzuzufügen, bearbeite STATIC_COLLISION_OBJECTS oben
    
    Beispiele:
    
    # Beispiel 1: Zur Home-Position fahren  
    move_to_home()
    
    # Beispiel 2: Um statische Hindernisse navigieren
    move_to_pose(0.3, 0.4, 0.4, 0.0, pi/2, 0.0)  # Links um Tisch
    move_to_pose(0.3, -0.4, 0.4, 0.0, pi/2, 0.0)  # Rechts um Tisch
    
    # MoveIt plant automatisch um alle definierten Collision Objects herum!
    
    """
    
    print("\n" + "═" * 70)
    print("  STUDENT PROGRAMM STARTET")
    print("═" * 70 + "\n")
    
    # ▼▼▼ DEIN CODE HIER ▼▼▼
    
    # Statische Collision Objects wurden automatisch beim Start geladen
    print("Collision Objects aus STATIC_COLLISION_OBJECTS sind aktiv!")
    
    # Beispiel: Fahre zur Home-Position
    move_to_home()
    print("Home erreicht!")
    
    # Beispiel: Bewege um den statischen Tisch herum
    print("Bewege seitlich um den Tisch...")
    move_to_pose(0.3, 0.4, 0.4, 0.0, pi/2, 0.0)  # Links um den Tisch
    print("Position links vom Tisch erreicht!")
    
    move_to_pose(0.3, -0.4, 0.4, 0.0, pi/2, 0.0)  # Rechts um den Tisch
    print("Position rechts vom Tisch erreicht!")
    
    # Zurück zur Home-Position
    move_to_home()
    
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
    print("═" * 70 + "\n")
    
    # ROS2 initialisieren
    rclpy.init()
    
    try:
        # Robot Controller erstellen
        _robot_instance = RobotController()
        
        # Student Programm ausführen
        student_program()
        
    except KeyboardInterrupt:
        print("\nProgramm durch Benutzer gestoppt (Ctrl+C)\n")
    except Exception as e:
        print(f"\nFehler: {str(e)}\n")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if _robot_instance is not None:
            _robot_instance.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
