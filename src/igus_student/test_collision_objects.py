#!/usr/bin/env python3
"""
Test-Script um Collision Objects zu visualisieren und zu testen
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene

class CollisionObjectTester(Node):
    
    def __init__(self):
        super().__init__("collision_object_tester")
        
        self.planning_scene_publisher = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )
        
        self.get_logger().info("Collision Object Tester gestartet")
        
        # Timer für periodisches Publizieren
        self.timer = self.create_timer(1.0, self.publish_test_objects)
        
    def publish_test_objects(self):
        """Publiziert Test-Collision-Objects"""
        try:
            planning_scene = PlanningScene()
            planning_scene.world.collision_objects = []
            
            # Test-Tisch
            table = CollisionObject()
            table.header.frame_id = "world"
            table.id = "test_table"
            
            # Box-Shape
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [1.2, 0.8, 0.02]  # width, depth, height
            
            # Position
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = 0.5
            pose.pose.position.y = 0.0
            pose.pose.position.z = -0.01
            pose.pose.orientation.w = 1.0
            
            table.primitives = [box]
            table.primitive_poses = [pose.pose]
            table.operation = CollisionObject.ADD
            
            # Test-Kugel
            sphere_obj = CollisionObject()
            sphere_obj.header.frame_id = "world"
            sphere_obj.id = "test_sphere"
            
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [0.1]  # radius
            
            sphere_pose = PoseStamped()
            sphere_pose.header.frame_id = "world"
            sphere_pose.pose.position.x = 0.3
            sphere_pose.pose.position.y = 0.3
            sphere_pose.pose.position.z = 0.2
            sphere_pose.pose.orientation.w = 1.0
            
            sphere_obj.primitives = [sphere]
            sphere_obj.primitive_poses = [sphere_pose.pose]
            sphere_obj.operation = CollisionObject.ADD
            
            planning_scene.world.collision_objects = [table, sphere_obj]
            planning_scene.is_diff = True
            
            self.planning_scene_publisher.publish(planning_scene)
            
            self.get_logger().info("📦 Test-Collision-Objects publiziert")
            self.get_logger().info("🔍 Überprüfen Sie RViz:")
            self.get_logger().info("   - Displays > PlanningScene > Scene Geometry > Show Scene Geometry: ✓")
            self.get_logger().info("   - Displays > MotionPlanning > Scene Geometry > Show Scene Geometry: ✓")
            
        except Exception as e:
            self.get_logger().error(f"Fehler beim Publizieren: {e}")

def main():
    rclpy.init()
    
    try:
        tester = CollisionObjectTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\nTester gestoppt")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()