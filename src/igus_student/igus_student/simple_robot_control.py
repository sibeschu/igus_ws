#!/usr/bin/env python3
"""
Simple IGUS Rebel Robot Control
Minimal interface for MoveIt motion planning with collision avoidance
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
from math import pi
import time

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint, 
    BoundingVolume, MotionPlanRequest, PlanningOptions,
    CollisionObject, PlanningScene, MoveItErrorCodes
)
from action_msgs.msg import GoalStatus
from scipy.spatial.transform import Rotation


# ═══════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════

PLANNING_GROUP = "igus_rebel_arm"
END_EFFECTOR_LINK = "link6"
REFERENCE_FRAME = "world"

POSITION_TOLERANCE = 0.01  # 10mm
ORIENTATION_TOLERANCE = 0.05  # ~3 degrees
VELOCITY_THRESHOLD = 0.05

HOME_POSITION = (0.4, 0.0, 0.3) # x, y, z
HOME_ORIENTATION = (pi, 0.0, 0.0)  # roll, pitch, yaw

# Static collision objects loaded at startup
COLLISION_OBJECTS = [
    {
        'name': 'table',
        'shape': 'box',
        'size': [2.5, 0.8, 0.02],
        'position': (0.0, 0.0, -0.01),
        'orientation': (0.0, 0.0, 0.0, 1.0)
    }
]


# ═══════════════════════════════════════════════════════════════════════════
# ROBOT CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__("simple_robot_controller")
        self.callback_group = ReentrantCallbackGroup()
        
        # MoveGroup action client
        self.move_client = ActionClient(
            self, MoveGroup, "/move_action", 
            callback_group=self.callback_group
        )
        
        # Planning scene publisher for collision objects
        self.scene_publisher = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )
        
        # Joint state subscriber for motion detection
        self.joint_velocities = None
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, 10
        )
        
        # Track if goal is currently executing
        self.is_goal_executing = False
        self.current_goal_handle = None
        
        # Wait for MoveGroup server
        self.get_logger().info("Connecting to MoveGroup...")
        if not self.move_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveGroup server not available!")
        
        self.get_logger().info("Connected to MoveGroup")
        
        # Add static collision objects
        self._add_collision_objects()
    
    def _joint_state_callback(self, msg):
        """Update joint velocities for motion detection"""
        if msg.velocity and len(msg.velocity) > 0:
            self.joint_velocities = np.array(msg.velocity)
    
  



    def _add_collision_objects(self):
        """Add all static collision objects to planning scene"""
        if not COLLISION_OBJECTS:
            return
        
        scene = PlanningScene()
        scene.world.collision_objects = []
        scene.is_diff = True
        
        for obj in COLLISION_OBJECTS:
            collision_obj = CollisionObject()
            collision_obj.header.frame_id = REFERENCE_FRAME
            collision_obj.id = obj['name']
            
            # Create shape
            primitive = SolidPrimitive()
            if obj['shape'] == 'box':
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [float(d) for d in obj['size']]
            elif obj['shape'] == 'sphere':
                primitive.type = SolidPrimitive.SPHERE
                primitive.dimensions = [float(d) for d in obj['size']]
            elif obj['shape'] == 'cylinder':
                primitive.type = SolidPrimitive.CYLINDER
                primitive.dimensions = [float(d) for d in obj['size']]
            
            # Create pose
            pose = PoseStamped()
            pose.header.frame_id = REFERENCE_FRAME
            pose.pose.position.x = float(obj['position'][0])
            pose.pose.position.y = float(obj['position'][1])
            pose.pose.position.z = float(obj['position'][2])
            pose.pose.orientation.x = float(obj['orientation'][0])
            pose.pose.orientation.y = float(obj['orientation'][1])
            pose.pose.orientation.z = float(obj['orientation'][2])
            pose.pose.orientation.w = float(obj['orientation'][3])
            
            collision_obj.primitives = [primitive]
            collision_obj.primitive_poses = [pose.pose]
            collision_obj.operation = CollisionObject.ADD
            
            scene.world.collision_objects.append(collision_obj)
            self.get_logger().info(f"Added collision object: {obj['name']}")
        
        # Publish scene
        self.scene_publisher.publish(scene)
        time.sleep(0.5)
        
        self.get_logger().info(f"Loaded {len(COLLISION_OBJECTS)} collision objects")

    def is_moving(self):
        """Check if robot is currently moving (no internal spin)"""
        if self.joint_velocities is None:
            return False
        return np.max(np.abs(self.joint_velocities)) > VELOCITY_THRESHOLD

    def wait_until_stopped(self, timeout=5.0, vel_thresh=VELOCITY_THRESHOLD, stable_time=0.5):
        """Block until velocities below threshold for stable_time or timeout."""
        start = time.time()
        stable_since = None
        last_vel = None
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.joint_velocities is None:
                continue
            last_vel = np.max(np.abs(self.joint_velocities))
            if last_vel < vel_thresh:
                if stable_since is None:
                    stable_since = time.time()
                if time.time() - stable_since >= stable_time:
                    self.get_logger().info(f"Robot settled (vel={last_vel:.3f})")
                    return True
            else:
                stable_since = None
        self.get_logger().warning(f"Timeout waiting for settle (last vel={last_vel:.3f})")
        return False

    def safe_move_to(self, x, y, z, roll, pitch, yaw):
        """
        Move robot end-effector to target pose
        
        Args:
            x, y, z: Position in meters
            roll, pitch, yaw: Orientation in radians
            
        Returns:
            bool: True if successful
        """
        self.get_logger().info(f"Target: ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # Convert euler to quaternion
        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
        quat = rotation.as_quat()  # [x, y, z, w]
        
        # Create target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = quat[0]
        target_pose.pose.orientation.y = quat[1]
        target_pose.pose.orientation.z = quat[2]
        target_pose.pose.orientation.w = quat[3]
        
        # Create position constraint
        sphere = SolidPrimitive(
            type=SolidPrimitive.SPHERE,
            dimensions=[POSITION_TOLERANCE]
        )
        position_constraint = PositionConstraint()
        position_constraint.header = target_pose.header
        position_constraint.link_name = END_EFFECTOR_LINK
        position_constraint.constraint_region = BoundingVolume(
            primitives=[sphere],
            primitive_poses=[target_pose.pose]
        )
        position_constraint.weight = 1.0
        
        # Create orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target_pose.header
        orientation_constraint.link_name = END_EFFECTOR_LINK
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE
        orientation_constraint.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE
        orientation_constraint.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE
        orientation_constraint.weight = 1.0
        
        # Create motion plan request
        motion_request = MotionPlanRequest()
        motion_request.group_name = PLANNING_GROUP
        motion_request.goal_constraints = [Constraints(
            position_constraints=[position_constraint],
            orientation_constraints=[orientation_constraint]
        )]

        # Max speeds
        motion_request.max_velocity_scaling_factor = 0.2
        motion_request.max_acceleration_scaling_factor = 0.2
        
        # Create move group goal
        goal = MoveGroup.Goal(
            request=motion_request,
            planning_options=PlanningOptions(plan_only=False)
        )
        
        try:
            # Check if already executing
            if self.is_goal_executing:
                self.get_logger().warning("Goal already executing! Waiting for completion...")
                return False
            
            # Send goal
            self.get_logger().info("Sending goal to MoveGroup...")
            self.is_goal_executing = True
            
            send_future = self.move_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            
            goal_handle = send_future.result()
            self.current_goal_handle = goal_handle
            
            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected by server")
                self.is_goal_executing = False
                self.current_goal_handle = None
                return False
            
            self.get_logger().info("Goal accepted, executing...")
            
            # Wait for result - BLOCKS until execution complete
            self.get_logger().info("Waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            # Reset execution state
            self.is_goal_executing = False
            self.current_goal_handle = None

            error_code = result.result.error_code.val
            if error_code == MoveItErrorCodes.SUCCESS:
                self.get_logger().info("Movement successful")
                # wait for physical settling
                self.get_logger().info("Waiting for robot to settle...")

                self.wait_until_stopped(timeout=5.0, stable_time=1)
                return True
            else:
                # MoveIt error codes from moveit_msgs/msg/MoveItErrorCodes
                error_messages = {
                    -1: "PLANNING_FAILED - Could not plan path",
                    -2: "INVALID_MOTION_PLAN - Generated plan is invalid",
                    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                    -4: "CONTROL_FAILED - Execution failed",
                    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                    -6: "TIMED_OUT - Planning/execution timeout",
                    -7: "PREEMPTED - Motion was preempted",
                    -10: "START_STATE_IN_COLLISION",
                    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
                    -12: "GOAL_IN_COLLISION",
                    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
                    -14: "GOAL_CONSTRAINTS_VIOLATED",
                    -15: "INVALID_GROUP_NAME",
                    -16: "INVALID_GOAL_CONSTRAINTS - Goal outside workspace",
                    -17: "INVALID_ROBOT_STATE",
                    -18: "INVALID_LINK_NAME",
                    -19: "INVALID_OBJECT_NAME",
                    -21: "FRAME_TRANSFORM_FAILURE",
                    -31: "NO_IK_SOLUTION - No inverse kinematics solution",
                    99999: "FAILURE - General failure"
                }
                msg = error_messages.get(error_code, f"Unknown error code {error_code}")
                self.get_logger().error(f"Movement failed: {msg}")
                return False

        except Exception as e:
            self.get_logger().error(f"Exception during movement: {e}")
            self.is_goal_executing = False
            self.current_goal_handle = None
            return False
    
    def is_executing_goal(self):
        """Check if a goal is currently being executed"""
        return self.is_goal_executing


# ═══════════════════════════════════════════════════════════════════════════
# USER INTERFACE
# ═══════════════════════════════════════════════════════════════════════════

_robot = None

def _get_robot():
    if _robot is None:
        raise RuntimeError("Robot not initialized")
    return _robot


def safe_move_to_pose(x, y, z, roll, pitch, yaw):
    """Move robot to target pose"""
    return _get_robot().safe_move_to(x, y, z, roll, pitch, yaw)


def move_home():
    """Move to home position"""
    x, y, z = HOME_POSITION
    roll, pitch, yaw = HOME_ORIENTATION
    return safe_move_to_pose(x, y, z, roll, pitch, yaw)


def is_robot_moving():
    """Check if robot is moving"""
    return _get_robot().is_moving()


def is_executing_goal():
    """Check if a MoveIt goal is currently being executed"""
    return _get_robot().is_executing_goal()


def wait_for_stop(timeout=3.0):
    """Wait until robot stops moving"""
    return _get_robot().wait_until_stopped(timeout)


# ═══════════════════════════════════════════════════════════════════════════
# MAIN PROGRAM
# ═══════════════════════════════════════════════════════════════════════════

def main():
    global _robot
    

    print("-" * 20 + "SIMPLE IGUS REBEL CONTROL" + "-" * 20)
  
    rclpy.init()
    
    try:
        _robot = SimpleRobotController()
        
        # ═══════════════════════════════════════════════════════════════════
        # YOUR PROGRAM HERE
        # ═══════════════════════════════════════════════════════════════════
        
        print("Starting motion sequence...")
        
        # Move to home - now automatically waits for robot to settle
        move_home()
        print("Reached home position")
        
        # Move to first position - automatically waits
        safe_move_to_pose(0.4, 0.2, 0.3, pi, 0.0, 0.0)
        print("Reached position 1")
        
        # Move to second position - automatically waits
        safe_move_to_pose(0.4, -0.2, 0.3, pi, 0.0, 0.0)
        print("Reached position 2")

        safe_move_to_pose(0.4, 0.0, 0.1, pi, 0.0, 0.0)
        print("Reached position 3")

        safe_move_to_pose(0.4, 0.1, 0.1, pi, 0.0, 0.0)
        print("Reached position 4")

        safe_move_to_pose(0.4, -0.3, 0.1, pi, 0.0, 0.0)
        print("Reached position 5")

        # Return home - automatically waits
        move_home()
        print("Returned home")
        
        # ═══════════════════════════════════════════════════════════════════
        
        print("-" * 20 + "  PROGRAM COMPLETED  " + "-" * 20)
        
    except KeyboardInterrupt:
        print("\nStopped by user (Ctrl+C)")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if _robot is not None:
            _robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
