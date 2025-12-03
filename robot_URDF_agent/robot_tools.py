# robot_tools.py
# ROS2 tools for controlling URDF robots with ROSA
# Compatible with ROS2 Jazzy

from math import sqrt
from typing import List, Dict, Any, Tuple
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

from langchain.agents import tool

# Module-level state
_cmd_vel_pubs: Dict[str, Any] = {}
_joint_pubs: Dict[str, Any] = {}
_helper_lock = threading.RLock()


class _ROS2Helper:
    """
    Lazy-initialized ROS2 helper for robot control operations.
    """
    _inited = False
    _node: Node | None = None

    @classmethod
    def ensure_init(cls):
        with _helper_lock:
            if not cls._inited:
                try:
                    rclpy.init(args=None)
                except Exception:
                    # rclpy may already be initialized
                    pass
                if cls._node is None:
                    cls._node = Node("rosa_robot_tools_node")
                cls._inited = True

    @classmethod
    def get_node(cls) -> Node:
        cls.ensure_init()
        assert cls._node is not None
        return cls._node

    @classmethod
    def create_publisher(cls, topic: str, msg_type, qos_profile=None):
        node = cls.get_node()
        if qos_profile is None:
            qos_profile = QoSProfile(depth=10)
        return node.create_publisher(msg_type, topic, qos_profile)

    @classmethod
    def wait_for_message(cls, topic: str, msg_type, timeout: float = 5.0):
        """
        Synchronously wait for a single message on `topic`.
        Returns the message or raises TimeoutError.
        """
        node = cls.get_node()
        got = {"msg": None}
        done = threading.Event()

        def callback(msg):
            got["msg"] = msg
            done.set()

        # Create QoS profile for best compatibility
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        sub = node.create_subscription(msg_type, topic, callback, qos)
        start = time.time()
        while not done.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.time() - start > timeout:
                node.destroy_subscription(sub)
                raise TimeoutError(f"Timeout waiting for message on {topic}")
        node.destroy_subscription(sub)
        return got["msg"]


def _ensure_cmd_vel_publisher(namespace: str = ""):
    """Ensure cmd_vel publisher exists."""
    with _helper_lock:
        if namespace not in _cmd_vel_pubs:
            topic = f"/{namespace}/cmd_vel" if namespace else "/cmd_vel"
            pub = _ROS2Helper.create_publisher(topic.replace("//", "/"), Twist)
            _cmd_vel_pubs[namespace] = pub


def _ensure_joint_publisher(controller_name: str = "joint_trajectory_controller"):
    """Ensure joint command publisher exists."""
    with _helper_lock:
        if controller_name not in _joint_pubs:
            topic = f"/{controller_name}/commands"
            pub = _ROS2Helper.create_publisher(topic, Float64MultiArray)
            _joint_pubs[controller_name] = pub


# ============================================================================
# ROBOT MOVEMENT TOOLS
# ============================================================================

def _move_robot_impl(linear_x: float, linear_y: float = 0.0, angular_z: float = 0.0, 
                     duration: float = 1.0, namespace: str = "") -> str:
    """Move the robot with specified velocities for a given duration.
    
    Args:
        linear_x: Forward/backward velocity (m/s)
        linear_y: Left/right velocity (m/s) - for holonomic robots
        angular_z: Rotational velocity (rad/s)
        duration: How long to apply the command (seconds)
        namespace: Robot namespace (if using multi-robot setup)
    
    Returns:
        Status message
    """
    _ensure_cmd_vel_publisher(namespace)
    
    twist = Twist()
    twist.linear.x = float(linear_x)
    twist.linear.y = float(linear_y)
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = float(angular_z)
    
    try:
        with _helper_lock:
            pub = _cmd_vel_pubs.get(namespace)
        
        node = _ROS2Helper.get_node()
        steps = max(1, int(duration * 10))  # 10Hz control loop
        
        for _ in range(steps):
            pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.1)
        
        # Stop the robot
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        
        return f"Robot moved: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z} for {duration}s"
    except Exception as e:
        return f"Failed to move robot: {e}"


def _stop_robot_impl(namespace: str = "") -> str:
    """Stop the robot by sending zero velocities.
    
    Args:
        namespace: Robot namespace
    
    Returns:
        Status message
    """
    return _move_robot_impl(0.0, 0.0, 0.0, 0.1, namespace)


def _rotate_robot_impl(angle_rad: float, angular_speed: float = 0.5, namespace: str = "") -> str:
    """Rotate the robot by a specified angle.
    
    Args:
        angle_rad: Angle to rotate (radians, positive = counter-clockwise)
        angular_speed: Rotational speed (rad/s)
        namespace: Robot namespace
    
    Returns:
        Status message
    """
    duration = abs(angle_rad) / angular_speed
    direction = angular_speed if angle_rad > 0 else -angular_speed
    
    return _move_robot_impl(0.0, 0.0, direction, duration, namespace)


# ============================================================================
# JOINT CONTROL TOOLS
# ============================================================================

def _set_joint_positions_impl(joint_positions: List[float], 
                               controller_name: str = "joint_trajectory_controller") -> str:
    """Set joint positions for the robot.
    
    Args:
        joint_positions: List of joint positions (radians or meters depending on joint type)
        controller_name: Name of the joint controller
    
    Returns:
        Status message
    """
    _ensure_joint_publisher(controller_name)
    
    msg = Float64MultiArray()
    msg.data = [float(pos) for pos in joint_positions]
    
    try:
        with _helper_lock:
            pub = _joint_pubs.get(controller_name)
        
        node = _ROS2Helper.get_node()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        
        return f"Set {len(joint_positions)} joint positions: {joint_positions}"
    except Exception as e:
        return f"Failed to set joint positions: {e}"


def _get_joint_states_impl(timeout: float = 5.0) -> Dict[str, Any]:
    """Get current joint states of the robot.
    
    Args:
        timeout: Maximum time to wait for joint states (seconds)
    
    Returns:
        Dictionary with joint names, positions, velocities, and efforts
    """
    try:
        msg = _ROS2Helper.wait_for_message("/joint_states", JointState, timeout)
        
        result = {
            "joint_names": list(msg.name),
            "positions": list(msg.position),
            "velocities": list(msg.velocity) if msg.velocity else [],
            "efforts": list(msg.effort) if msg.effort else []
        }
        
        return result
    except TimeoutError:
        raise RuntimeError("Failed to get joint states: /joint_states topic not available")
    except Exception as e:
        raise RuntimeError(f"Failed to get joint states: {e}")


# ============================================================================
# POSE AND ODOMETRY TOOLS
# ============================================================================

def _get_robot_pose_impl(source: str = "odom", timeout: float = 5.0) -> Dict[str, float]:
    """Get current robot pose from odometry or localization.
    
    Args:
        source: Source of pose data ('odom', 'amcl', or 'ground_truth')
        timeout: Maximum time to wait (seconds)
    
    Returns:
        Dictionary with x, y, z positions and orientation quaternion
    """
    topic_map = {
        "odom": "/odom",
        "amcl": "/amcl_pose",
        "ground_truth": "/ground_truth/odom"
    }
    
    topic = topic_map.get(source, "/odom")
    
    try:
        if "odom" in topic:
            msg = _ROS2Helper.wait_for_message(topic, Odometry, timeout)
            pose = msg.pose.pose
        else:
            msg = _ROS2Helper.wait_for_message(topic, PoseStamped, timeout)
            pose = msg.pose
        
        return {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": pose.orientation.x,
            "qy": pose.orientation.y,
            "qz": pose.orientation.z,
            "qw": pose.orientation.w
        }
    except TimeoutError:
        raise RuntimeError(f"Failed to get robot pose: {topic} not available")
    except Exception as e:
        raise RuntimeError(f"Failed to get robot pose: {e}")


def _check_position_reached_impl(target_x: float, target_y: float, 
                                  tolerance: float = 0.1, source: str = "odom") -> str:
    """Check if robot has reached a target position.
    
    Args:
        target_x: Target X coordinate (meters)
        target_y: Target Y coordinate (meters)
        tolerance: Acceptable distance from target (meters)
        source: Pose source ('odom', 'amcl', 'ground_truth')
    
    Returns:
        Status message indicating if target reached
    """
    try:
        pose = _get_robot_pose_impl(source)
        distance = sqrt((pose["x"] - target_x)**2 + (pose["y"] - target_y)**2)
        
        if distance <= tolerance:
            return f"Target reached! Current position: ({pose['x']:.3f}, {pose['y']:.3f}), distance: {distance:.3f}m"
        else:
            return f"Target not reached. Current position: ({pose['x']:.3f}, {pose['y']:.3f}), distance: {distance:.3f}m"
    except Exception as e:
        return f"Failed to check position: {e}"


# ============================================================================
# LANGCHAIN TOOL WRAPPERS
# ============================================================================

move_robot = tool(_move_robot_impl)
stop_robot = tool(_stop_robot_impl)
rotate_robot = tool(_rotate_robot_impl)
set_joint_positions = tool(_set_joint_positions_impl)
get_joint_states = tool(_get_joint_states_impl)
get_robot_pose = tool(_get_robot_pose_impl)
check_position_reached = tool(_check_position_reached_impl)
