

from math import cos, sin, sqrt
from typing import List, Dict, Tuple, Any
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, TeleportRelative, SetPen
from std_srvs.srv import Empty

from langchain.agents import tool  # same decorator used previously

# Module-level state
_cmd_vel_pubs: Dict[str, Any] = {}
_helper_lock = threading.RLock()


class _ROS2Helper:
    """
    Lazy-initialized ROS2 helper that owns a single Node used for
    synchronous operations (publishers, clients, temporary subscriptions).
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
                    # rclpy may already be initialized by the caller; ignore errors
                    pass
                if cls._node is None:
                    cls._node = Node("rosa_turtle_tools_node")
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
    def wait_for_service(cls, client, timeout_sec: float = 5.0) -> bool:
        # client is rclpy client
        try:
            return client.wait_for_service(timeout_sec=timeout_sec)
        except Exception:
            return False

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

        sub = node.create_subscription(msg_type, topic, callback, 10)
        start = time.time()
        while not done.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.time() - start > timeout:
                # clean up subscription
                node.destroy_subscription(sub)
                raise TimeoutError(f"Timeout waiting for message on {topic}")
        # clean up subscription
        node.destroy_subscription(sub)
        return got["msg"]

    @classmethod
    def call_service(cls, srv_type, srv_name: str, request, timeout_sec: float = 5.0):
        """
        Call a service synchronously. Returns the response or raises RuntimeError on failure.
        """
        node = cls.get_node()
        client = node.create_client(srv_type, srv_name)
        if not cls.wait_for_service(client, timeout_sec=timeout_sec):
            node.destroy_client(client)
            raise RuntimeError(f"Service {srv_name} not available")
        future = client.call_async(request)
        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.time() - start > timeout_sec:
                node.destroy_client(client)
                raise RuntimeError(f"Timeout calling service {srv_name}")
        resp = future.result()
        node.destroy_client(client)
        return resp


# Utility functions (bounds)
def within_bounds(x: float, y: float) -> Tuple[bool, str]:
    if 0 <= x <= 11 and 0 <= y <= 11:
        return True, "Coordinates are within bounds."
    else:
        return False, f"({x}, {y}) will be out of bounds. Range is [0, 11] for each."


# Internal implementations (not tool-wrapped); we will wrap them with @tool equivalents below.

def _ensure_default_publisher():
    """Ensure /turtle1/cmd_vel publisher exists in mapping."""
    with _helper_lock:
        if "turtle1" not in _cmd_vel_pubs:
            pub = _ROS2Helper.create_publisher("/turtle1/cmd_vel", Twist)
            _cmd_vel_pubs["turtle1"] = pub


def _will_be_within_bounds(
    name: str, velocity: float, lateral: float, angle: float, duration: float = 1.0
) -> Tuple[bool, str]:
    # Use get pose implementation to obtain current pose
    try:
        poses = _get_turtle_pose_impl([name])
    except Exception as e:
        return False, f"Unable to determine current pose: {e}"

    current_x = poses[name].x
    current_y = poses[name].y
    current_theta = poses[name].theta

    # Straight line
    if abs(angle) < 1e-6:
        new_x = current_x + (velocity * cos(current_theta) - lateral * sin(current_theta)) * duration
        new_y = current_y + (velocity * sin(current_theta) + lateral * cos(current_theta)) * duration
    else:
        radius = sqrt(max(velocity**2 + lateral**2, 1e-12)) / abs(angle)
        center_x = current_x - radius * sin(current_theta)
        center_y = current_y + radius * cos(current_theta)
        angle_traveled = angle * duration
        new_x = center_x + radius * sin(current_theta + angle_traveled)
        new_y = center_y - radius * cos(current_theta + angle_traveled)

        for t in range(max(1, int(duration) + 1)):
            angle_t = current_theta + angle * t
            x_t = center_x + radius * sin(angle_t)
            y_t = center_y - radius * cos(angle_t)
            in_bounds, _ = within_bounds(x_t, y_t)
            if not in_bounds:
                return False, f"The circular path will go out of bounds at ({x_t:.2f}, {y_t:.2f})."

    in_bounds, message = within_bounds(new_x, new_y)
    if not in_bounds:
        return False, f"This command will move the turtle out of bounds to ({new_x:.2f}, {new_y:.2f})."
    return True, f"The turtle will remain within bounds at ({new_x:.2f}, {new_y:.2f})."


# Implementation functions (core logic)
def _spawn_turtle_impl(name: str, x: float, y: float, theta: float) -> str:
    """Spawn a new turtle named `name` at coordinates `(x, y, theta)`.

    Returns a human-readable status string indicating success or failure.
    """
    in_bounds, message = within_bounds(x, y)
    if not in_bounds:
        return message

    name = name.replace("/", "")
    _ROS2Helper.ensure_init()
    req = Spawn.Request()
    req.x = float(x)
    req.y = float(y)
    req.theta = float(theta)
    req.name = name

    try:
        _ROS2Helper.call_service(Spawn, "/spawn", req, timeout_sec=5.0)
        # create publisher for the new turtle
        with _helper_lock:
            pub = _ROS2Helper.create_publisher(f"/{name}/cmd_vel", Twist)
            _cmd_vel_pubs[name] = pub
        return f"{name} spawned at x: {x}, y: {y}, theta: {theta}."
    except Exception as e:
        return f"Failed to spawn {name}: {e}"


def _kill_turtle_impl(names: List[str]) -> str:
    """Kill one or more turtles by name.

    `names` may be a list of turtle names (strings). Returns status messages.
    """
    names = [n.replace("/", "") for n in names]
    responses = []
    for name in names:
        try:
            req = Kill.Request()
            # no fields required
            _ROS2Helper.call_service(Kill, f"/{name}/kill", req, timeout_sec=5.0)
            with _helper_lock:
                _cmd_vel_pubs.pop(name, None)
            responses.append(f"Successfully killed {name}.")
        except Exception as e:
            responses.append(f"Failed to kill {name}: {e}")
    return "\n".join(responses)


def _clear_turtlesim_impl() -> str:
    """Clear the turtlesim background (call the `/clear` service).

    Returns a human-readable status string indicating success or failure.
    """
    try:
        req = Empty.Request()
        _ROS2Helper.call_service(Empty, "/clear", req, timeout_sec=5.0)
        return "Successfully cleared the turtlesim background."
    except Exception as e:
        return f"Failed to clear the turtlesim background: {e}"


def _get_turtle_pose_impl(names: List[str]) -> dict:
    """Get the current `Pose` message for the given turtle names.

    Returns a dict mapping turtle name -> `turtlesim.msg.Pose`.
    """
    names = [n.replace("/", "") for n in names]
    poses = {}
    for name in names:
        topic = f"/{name}/pose"
        try:
            msg = _ROS2Helper.wait_for_message(topic, Pose, timeout=5.0)
            poses[name] = msg
        except TimeoutError:
            raise RuntimeError(f"Failed to get pose for {name}: {topic} not available.")
    return poses


def _teleport_absolute_impl(name: str, x: float, y: float, theta: float, hide_pen: bool = True) -> str:
    """Teleport the turtle `name` to absolute coordinates `(x, y, theta)`.

    If `hide_pen` is True, temporarily disables the pen while teleporting.
    Returns a status string or error.
    """
    in_bounds, message = within_bounds(x, y)
    if not in_bounds:
        return message

    name = name.replace("/", "")
    try:
        if hide_pen:
            # hide pen
            _set_pen_impl(name=name, r=0, g=0, b=0, width=1, off=1)
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        _ROS2Helper.call_service(TeleportAbsolute, f"/{name}/teleport_absolute", req, timeout_sec=5.0)
        if hide_pen:
            _set_pen_impl(name=name, r=30, g=30, b=255, width=1, off=0)
        current_pose = _get_turtle_pose_impl([name])
        p = current_pose[name]
        return f"{name} new pose: ({p.x}, {p.y}) at {p.theta} radians."
    except Exception as e:
        return f"Failed to teleport the turtle: {e}"


def _teleport_relative_impl(name: str, linear: float, angular: float) -> str:
    """Teleport the turtle `name` by `linear` (forward) and `angular` (radians).

    Returns the new pose or an error message.
    """
    name = name.replace("/", "")
    in_bounds, message = _will_be_within_bounds(name, linear, 0.0, angular)
    if not in_bounds:
        return message
    try:
        req = TeleportRelative.Request()
        req.linear = float(linear)
        req.angular = float(angular)
        _ROS2Helper.call_service(TeleportRelative, f"/{name}/teleport_relative", req, timeout_sec=5.0)
        current_pose = _get_turtle_pose_impl([name])
        p = current_pose[name]
        return f"{name} new pose: ({p.x}, {p.y}) at {p.theta} radians."
    except Exception as e:
        return f"Failed to teleport the turtle: {e}"


def _publish_twist_to_cmd_vel_impl(name: str, velocity: float, lateral: float, angle: float, steps: int = 1) -> str:
    """Publish a `Twist` to `/<name>/cmd_vel` with the given linear/lateral/angle.

    `steps` controls how many times the message is published (duration proxy).
    Returns the current pose after publishing.
    """
    name = name.replace("/", "")
    _ensure_default_publisher()
    in_bounds, message = _will_be_within_bounds(name, velocity, lateral, angle, duration=steps)
    if not in_bounds:
        return message

    vel = Twist()
    vel.linear.x = float(velocity)
    vel.linear.y = float(lateral)
    vel.linear.z = 0.0
    vel.angular.x = 0.0
    vel.angular.y = 0.0
    vel.angular.z = float(angle)

    try:
        with _helper_lock:
            pub = _cmd_vel_pubs.get(name)
            if pub is None:
                # create one on demand
                pub = _ROS2Helper.create_publisher(f"/{name}/cmd_vel", Twist)
                _cmd_vel_pubs[name] = pub

        node = _ROS2Helper.get_node()
        for _ in range(max(1, int(steps))):
            pub.publish(vel)
            # spin once and sleep a bit so the message goes out and simulator updates
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.1)
    except Exception as e:
        return f"Failed to publish {vel} to /{name}/cmd_vel: {e}"
    # return current pose
    current_pose = _get_turtle_pose_impl([name])
    p = current_pose[name]
    lin_vel = getattr(p, "linear_velocity", None)
    ang_vel = getattr(p, "angular_velocity", None)
    return (
        f"New Pose ({name}): x={p.x}, y={p.y}, theta={p.theta} rads, "
        f"linear_velocity={lin_vel}, angular_velocity={ang_vel}."
    )


def _stop_turtle_impl(name: str) -> str:
    """Stop the named turtle by publishing zero velocities.

    Returns the pose/status string from the publish helper.
    """
    return _publish_twist_to_cmd_vel_impl(name=name, velocity=0.0, lateral=0.0, angle=0.0, steps=1)


def _reset_turtlesim_impl() -> str:
    """Reset the turtlesim environment by calling the `/reset` service.

    Clears internal publisher cache and re-creates the `/turtle1/cmd_vel` publisher.
    Returns a status string.
    """
    try:
        req = Empty.Request()
        _ROS2Helper.call_service(Empty, "/reset", req, timeout_sec=5.0)
        with _helper_lock:
            _cmd_vel_pubs.clear()
            _cmd_vel_pubs["turtle1"] = _ROS2Helper.create_publisher("/turtle1/cmd_vel", Twist)
        return "Successfully reset the turtlesim environment. Ignore all previous commands, failures, and goals."
    except Exception as e:
        return f"Failed to reset the turtlesim environment: {e}"


def _set_pen_impl(name: str, r: int, g: int, b: int, width: int, off: int) -> str:
    """Set the pen color/width for turtle `name` using the `/set_pen` service.

    `off` should be 1 to disable the pen, 0 to enable.
    Returns a status string.
    """
    name = name.replace("/", "")
    try:
        req = SetPen.Request()
        req.r = int(r)
        req.g = int(g)
        req.b = int(b)
        req.width = int(width)
        req.off = int(off)
        _ROS2Helper.call_service(SetPen, f"/{name}/set_pen", req, timeout_sec=5.0)
        return f"Successfully set the pen color for the turtle: {name}."
    except Exception as e:
        return f"Failed to set the pen color for the turtle: {e}"


def _has_moved_to_expected_coordinates_impl(name: str, expected_x: float, expected_y: float, tolerance: float = 0.1) -> str:
    """Check whether `name` is within `tolerance` of (`expected_x`, `expected_y`).

    Returns a human-readable pass/fail message.
    """
    current_pose = _get_turtle_pose_impl([name])
    p = current_pose[name]
    current_x = p.x
    current_y = p.y
    distance = sqrt((current_x - expected_x) ** 2 + (current_y - expected_y) ** 2)
    if distance <= tolerance:
        return f"{name} has moved to the expected position ({expected_x}, {expected_y})."
    else:
        return f"{name} has NOT moved to the expected position ({expected_x}, {expected_y})."


# Create langchain tool wrappers (these return Tool objects with .invoke())
spawn_turtle = tool(_spawn_turtle_impl)
kill_turtle = tool(_kill_turtle_impl)
clear_turtlesim = tool(_clear_turtlesim_impl)
get_turtle_pose = tool(_get_turtle_pose_impl)
teleport_absolute = tool(_teleport_absolute_impl)
teleport_relative = tool(_teleport_relative_impl)
publish_twist_to_cmd_vel = tool(_publish_twist_to_cmd_vel_impl)
stop_turtle = tool(_stop_turtle_impl)
reset_turtlesim = tool(_reset_turtlesim_impl)
set_pen = tool(_set_pen_impl)
has_moved_to_expected_coordinates = tool(_has_moved_to_expected_coordinates_impl)


# Do not create ROS publishers at import time to avoid unintended rclpy initialization.
# Publishers will be created lazily when the first tool runs and rclpy is properly initialized.

