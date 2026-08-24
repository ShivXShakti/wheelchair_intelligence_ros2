#!/usr/bin/env python3
"""
Intelligence → Nav2 Communication Bridge
=========================================
Subscribes to /llm_command (published by the whisper_llm backend) and
translates higher-level action commands into Nav2 NavigateToPose goals.

Supported actions (JSON on /llm_command):
  {"action": "navigate", "destination": "<location_key>"}
  {"action": "stop"}
  {"action": "wait"}
  {"action": "resume"}

Publishes navigation status on /nav_status as JSON:
  {"status": "navigating|paused|idle|reached|failed|cancelled",
   "destination": "...", "distance_remaining": ...}

Improvements over e_hla_navigate2pose.py:
  - No duplicate imports
  - Non-blocking Nav2 server discovery (retries in background)
  - ROS2 parameter for YAML config path (overridable via launch)
  - /nav_status publisher so the UI/backend knows nav state
  - Robust YAML key validation (no KeyError crashes)
  - Proper cancel→wait→resend sequencing (no race conditions)
  - Throttled feedback logging (every 2s instead of every tick)
  - Graceful SIGINT / destroy_node handling
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterDescriptor
from action_msgs.srv import CancelGoal

import json
import math
import yaml
import os
import time
import signal
import sys

from ament_index_python.packages import get_package_share_directory


# ──────────────────────────────────────────────────────────────
#  Navigation state enum (string-based for JSON serialisation)
# ──────────────────────────────────────────────────────────────
class NavState:
    IDLE       = "idle"
    NAVIGATING = "navigating"
    PAUSED     = "paused"
    REACHED    = "reached"
    FAILED     = "failed"
    CANCELLED  = "cancelled"


class IntelligenceToNav2(Node):
    """ROS2 node bridging LLM commands to the Nav2 NavigateToPose action."""

    def __init__(self):
        super().__init__('intelligence_to_nav2_com')

        # ── ROS2 Parameters ──────────────────────────────────
        self.declare_parameter(
            'semantics_yaml', '',
            descriptor=ParameterDescriptor(
                description='Absolute path to the semantic map YAML. '
                            'If empty, defaults to <pkg_share>/config/map_semantics.yaml'
            )
        )
        self.declare_parameter(
            'feedback_log_interval', 2.0,
            descriptor=ParameterDescriptor(
                description='Minimum interval (seconds) between feedback log messages'
            )
        )

        # ── Load semantic locations ──────────────────────────
        self.destinations: dict = self._load_locations()
        if not self.destinations:
            self.get_logger().warn(
                'No destinations loaded — navigate commands will be rejected '
                'until a valid YAML is provided.'
            )

        # ── State tracking ───────────────────────────────────
        self.nav_state: str         = NavState.IDLE
        self.current_goal_handle    = None
        self.active_goal_pose: PoseStamped | None  = None
        self.paused_goal_pose: PoseStamped | None  = None
        self.current_destination: str              = ""
        self._last_feedback_time: float            = 0.0
        self._last_distance: float                 = float('inf')
        self._server_ready: bool                   = False

        # ── Callback group (allow concurrent callbacks) ──────
        self._cb_group = ReentrantCallbackGroup()

        # ── Nav2 Action Client ───────────────────────────────
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self._cb_group
        )

        # ── Subscribers ──────────────────────────────────────
        self.create_subscription(
            String, '/llm_command', self._llm_command_cb, 10,
            callback_group=self._cb_group
        )
        
        # ── Global Cancel Service ────────────────────────────
        self.cancel_client = self.create_client(
            CancelGoal, 
            '/navigate_to_pose/_action/cancel_goal',
            callback_group=self._cb_group
        )

        # ── Publishers ───────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.feedback_pub = self.create_publisher(String, '/nav_feedback', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Non-blocking server discovery ────────────────────
        self._discovery_timer = self.create_timer(
            2.0, self._check_nav2_server, callback_group=self._cb_group
        )
        self.get_logger().info(
            'Intelligence→Nav2 bridge initialised. '
            'Waiting for Nav2 action server in background...'
        )

    # ==========================================================
    #  Initialisation helpers
    # ==========================================================
    def _load_locations(self) -> dict:
        """Load semantic map YAML, with fallback to package share dir."""
        yaml_path = self.get_parameter('semantics_yaml').get_parameter_value().string_value

        if not yaml_path:
            search_paths = []
            
            # 1. Attempt to pull from centralized backend_config.json dynamically!
            config_paths = [
                "/wheelchair_ws/wheelchair_intelligence/development/config/backend_config.json",
                "/wheelchair_ws/google_quant/wheelchair_intelligence/development/config/backend_config.json",
                "/home/robot/wheelchair_ws/google_quant/wheelchair_intelligence/development/config/backend_config.json",
                "/home/ducky/wheelchair_ws/wheelchair_intelligence/development/config/backend_config.json",
                "/home/container_user/wheelchair2/dependencies/wheelchair_intelligence_ros2/config/backend_config.json",
                "/home/container_user/wheelchair2/src/dependencies/wheelchair_intelligence_ros2/config/backend_config.json"
            ]
            config_data = {}
            for path in config_paths:
                if os.path.isfile(path):
                    try:
                        with open(path, 'r') as f:
                            config_data = json.load(f)
                        self.get_logger().info(f'Loaded config from {path}')
                        break
                    except Exception as e:
                        self.get_logger().error(f'Failed to parse config {path}: {e}')

            if config_data.get("map_semantics_path"):
                search_paths.append(config_data.get("map_semantics_path"))
            search_paths.extend(config_data.get("map_semantics_candidates", []))
            
            # 2. Check current OS Environment Variable directly
            if 'MAP_SEMANTICS_PATH' in os.environ:
                search_paths.append(os.path.expanduser(os.environ['MAP_SEMANTICS_PATH']))

            # 3. Package share directory (colcon install) fallback
            try:
                pkg_share = get_package_share_directory('wheelchair_intelligence_ros2')
                search_paths.append(os.path.join(pkg_share, 'config', 'map_semantics_dqn4.yaml'))
            except Exception:
                pass

            # Container / alternate workspace fallback
            search_paths.append(
                '/wheelchair_ws/wheelchair_intelligence/development/map_semantics.yaml'
            )
            search_paths.append(
                '/wheelchair_ws/wheelchair_intelligence/development/config/map_semantics.yaml'
            )
            search_paths.append(
                '/wheelchair_ws/google_quant/wheelchair_intelligence/development/map_semantics.yaml'
            )
            search_paths.append(
                '/wheelchair_ws/google_quant/wheelchair_intelligence/development/config/map_semantics.yaml'
            )
            search_paths.append(
                '/wheelchair_ws/wheelchair_intelligence/map_semantics.yaml'
            )
            search_paths.append(
                '/home/container_user/wheelchair2/dependencies/'
                'wheelchair_intelligence_ros2/config/map_semantics.yaml'
            )
            search_paths.append(
                '/home/container_user/wheelchair2/src/dependencies/'
                'wheelchair_intelligence_ros2/config/map_semantics.yaml'
            )
            search_paths.append(
                '/home/robot/wheelchair_ws/ros2_ws/src/'
                'wheelchair_intelligence_ros2/config/map_semantics.yaml'
            )
            search_paths.append(
                '/home/robot/wheelchair_ws/wheelchair2/dependencies/'
                'wheelchair_intelligence_ros2/config/map_semantics.yaml'
            )
            search_paths.append(
                '/home/container_user/wheelchair2/src/'
                'wheelchair_intelligence_ros2/config/map_semantics.yaml'
            )

            for candidate in search_paths:
                if os.path.isfile(candidate):
                    yaml_path = candidate
                    break

            if not yaml_path:
                self.get_logger().error(
                    'Could not find map_semantics.yaml in any search path: '
                    f'{search_paths}. Set the semantics_yaml parameter explicitly.'
                )
                return {}

        self.get_logger().info(f'Loading locations from: {yaml_path}')

        if not os.path.isfile(yaml_path):
            self.get_logger().error(f'Semantics YAML not found: {yaml_path}')
            return {}

        try:
            with open(yaml_path, 'r') as f:
                raw = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f'Failed to parse YAML: {e}')
            return {}

        # Validate each entry has the required pose keys
        validated: dict = {}
        required_keys = {'x', 'y', 'yaw'}
        for name, data in raw.items():
            if not isinstance(data, dict):
                self.get_logger().warn(f'Skipping "{name}": value is not a dict')
                continue
            missing = required_keys - set(data.keys())
            if missing:
                self.get_logger().warn(
                    f'Skipping "{name}": missing keys {missing}'
                )
                continue
            validated[name] = data

        self.get_logger().info(
            f'Loaded {len(validated)} valid destinations '
            f'(skipped {len(raw) - len(validated)})'
        )
        return validated

    def _check_nav2_server(self):
        """Periodically probe the Nav2 action server until it appears."""
        if self._server_ready:
            return
        if self.action_client.server_is_ready():
            self._server_ready = True
            self._discovery_timer.cancel()
            self.get_logger().info('Nav2 action server is READY ✓')
            self._publish_status()
        else:
            self.get_logger().warn('Nav2 is NOT up — retrying in 2s...')

    # ==========================================================
    #  /llm_command callback
    # ==========================================================
    def _llm_command_cb(self, msg: String):
        """Parse the JSON command and dispatch to the appropriate handler."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON on /llm_command: {msg.data}')
            return

        action      = str(data.get('action', '')).lower().strip()
        destination = str(data.get('destination', '')).lower().strip()

        self.get_logger().info(f'[CMD] action="{action}" destination="{destination}"')

        dispatch = {
            'stop':     self._handle_stop,
            'wait':     self._handle_pause,
            'resume':   self._handle_resume,
            'navigate': lambda: self._handle_navigate(destination),
        }

        handler = dispatch.get(action)
        if handler:
            handler()
        else:
            self.get_logger().debug(f'Ignoring unhandled action: "{action}"')

    # ==========================================================
    #  Command handlers
    # ==========================================================
    def _handle_navigate(self, destination: str):
        """Resolve destination and send a NavigateToPose goal."""
        if not self._server_ready:
            self.get_logger().warn('Nav2 server not ready — ignoring navigate command')
            return

        # Hot-reload locations to instantly pick up newly saved poses
        self.destinations = self._load_locations()

        # Case-insensitive and normalized key matching
        matched_key = None
        for key, data in self.destinations.items():
            k_norm = key.lower().strip()
            d_norm = destination.lower().strip()
            aliases = [a.lower().strip() for a in data.get("aliases", [])] if isinstance(data, dict) else []
            if k_norm == d_norm or k_norm.replace("_", " ") == d_norm.replace("_", " ") or d_norm in aliases:
                matched_key = key
                break

        if not matched_key:
            self.get_logger().warn(
                f'Unknown destination "{destination}". '
                f'Available: {list(self.destinations.keys())}'
            )
            return

        destination = matched_key

        goal_data = self.destinations[destination]
        pose = self._build_pose(goal_data)

        self.current_destination = destination
        self.active_goal_pose    = pose
        self.paused_goal_pose    = None

        self._send_goal(pose)

    def _handle_stop(self):
        """Cancel current goal and clear all saved poses."""
        self.get_logger().info('Stopping navigation (Cancelling ALL goals)...')
        
        # Remember the goal in case the user says "continue" later
        if self.active_goal_pose:
            self.paused_goal_pose = self.active_goal_pose
        self.active_goal_pose = None
        self.nav_state = NavState.CANCELLED
        self._publish_status()

        # 1. Cancel our local goal handle if we have one
        if self.current_goal_handle is not None:
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(self._on_cancel_done)
            
        # 2. Force cancel ALL goals globally (handles RViz2 goals we don't track)
        if self.cancel_client.wait_for_service(timeout_sec=0.5):
            req = CancelGoal.Request()
            req.goal_info.goal_id.uuid = [0] * 16 # All zeros = cancel all
            self.cancel_client.call_async(req)
        else:
            self.get_logger().warn('Cancel service unavailable.')

        # 3. Halt physical motion immediately
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def _handle_pause(self):
        """Pause (cancel the current goal but remember it for resume)."""
        if self.current_goal_handle is None:
            self.get_logger().info('Nothing to pause.')
            return

        self.get_logger().info('Pausing navigation...')
        self.paused_goal_pose = self.active_goal_pose
        self.nav_state = NavState.PAUSED

        future = self.current_goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_cancel_done)

    def _handle_resume(self):
        """Resume a previously paused goal."""
        if self.paused_goal_pose is None:
            self.get_logger().info('No paused goal to resume.')
            return
        if not self._server_ready:
            self.get_logger().warn('Nav2 server not ready — cannot resume')
            return

        self.get_logger().info(f'Resuming navigation to "{self.current_destination}"...')
        self.active_goal_pose = self.paused_goal_pose
        self.paused_goal_pose = None
        self._send_goal(self.active_goal_pose)

    # ==========================================================
    #  Nav2 goal lifecycle
    # ==========================================================
    def _send_goal(self, pose: PoseStamped):
        """Cancel any existing goal, then send a new one."""
        if self.current_goal_handle is not None:
            self.get_logger().info('Cancelling previous goal before sending new one...')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda f: self._dispatch_goal(pose)
            )
            return
        self._dispatch_goal(pose)

    def _dispatch_goal(self, pose: PoseStamped):
        """Actually send the NavigateToPose goal."""
        # Refresh the timestamp so Nav2 doesn't reject a stale pose
        pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_state = NavState.NAVIGATING
        self._last_feedback_time = 0.0
        self._last_distance = float('inf')
        self._publish_status()

        self.get_logger().info(
            f'Sending goal → x={pose.pose.position.x:.2f}, '
            f'y={pose.pose.position.y:.2f}'
        )

        send_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb
        )
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        """Called when Nav2 accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal REJECTED by Nav2')
            self.nav_state = NavState.IDLE
            self.current_goal_handle = None
            self._publish_status_string("Rejected")
            self._publish_status()
            return

        self.get_logger().info('Goal ACCEPTED ✓')
        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        """Called when navigation finishes (success, failure, or abort)."""
        status = future.result().status

        # action_msgs/GoalStatus constants
        # 4 = STATUS_SUCCEEDED, 5 = STATUS_CANCELED, 6 = STATUS_ABORTED
        if status == 4:
            self.nav_state = NavState.REACHED
            self.get_logger().info(
                f'Navigation SUCCEEDED — arrived at "{self.current_destination}"'
            )
            self._publish_status_string("SUCCEEDED")
        elif status == 5:
            # Already set to CANCELLED or PAUSED by the handler
            if self.nav_state not in (NavState.PAUSED, NavState.CANCELLED):
                self.nav_state = NavState.CANCELLED
            self.get_logger().info('Navigation CANCELLED')
            self._publish_status_string("CANCELLED")
        else:
            self.nav_state = NavState.FAILED
            self.get_logger().warn(f'Navigation FAILED (status={status})')
            self._publish_status_string("Failed")

        self.current_goal_handle = None
        self._publish_status()

    def _on_cancel_done(self, future):
        """Generic callback after a cancel request completes."""
        self.current_goal_handle = None
        self.get_logger().info(f'Cancel acknowledged — state={self.nav_state}')
        self._publish_status()

    def _feedback_cb(self, feedback_msg):
        """Throttled feedback logging to avoid terminal spam."""
        distance = feedback_msg.feedback.distance_remaining
        self._last_distance = distance

        now = time.monotonic()
        interval = self.get_parameter(
            'feedback_log_interval'
        ).get_parameter_value().double_value

        if (now - self._last_feedback_time) >= interval:
            self._last_feedback_time = now
            self.get_logger().info(f'Distance remaining: {distance:.2f} m')
            
            fb_msg = String()
            fb_msg.data = f"{distance:.2f}"
            self.feedback_pub.publish(fb_msg)
            
            self._publish_status()

    # ==========================================================
    #  Status publishing
    # ==========================================================
    def _publish_status_string(self, status_str: str):
        """Broadcast simple string status for backend compatibility."""
        status_msg = String()
        status_msg.data = status_str
        self.status_pub.publish(status_msg)

    def _publish_status(self):
        """Broadcast current navigation state on /navigation_status."""
        status_msg = String()
        status_msg.data = json.dumps({
            'status': self.nav_state,
            'destination': self.current_destination,
            'distance_remaining': round(self._last_distance, 2)
                if self._last_distance != float('inf') else -1,
            'server_ready': self._server_ready,
        })
        self.status_pub.publish(status_msg)

    # ==========================================================
    #  Utility
    # ==========================================================
    @staticmethod
    def _build_pose(goal_data: dict) -> PoseStamped:
        """Create a PoseStamped from a YAML location entry."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        # stamp is set right before sending

        pose.pose.position.x = float(goal_data['x'])
        pose.pose.position.y = float(goal_data['y'])
        pose.pose.position.z = 0.0

        yaw = float(goal_data['yaw'])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose


# ==============================================================
#  Entry point
# ==============================================================
def main(args=None):
    rclpy.init(args=args)
    node = IntelligenceToNav2()

    # Removed custom SIGINT handler to prevent deadlock with rclpy.spin

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
