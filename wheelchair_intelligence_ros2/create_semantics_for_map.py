#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import yaml
import os
import math


class Nav2PlanSaver(Node):

    def __init__(self):
        super().__init__('nav2_plan_saver')

        self.yaml_file = os.path.expanduser(
            "/home/robot/wheelchair_ws/google_quant/ros2_ws/src/wheelchair_intelligence_ros2/config/rrclab_map_semantics.yaml"
        )

        self.new_goal = None
        self.last_saved = None

        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10
        )

        self.timer = self.create_timer(0.5, self.process_goal)

        self.get_logger().info("Listening to Nav2 /plan safely...")

    def plan_callback(self, msg):

        if len(msg.poses) == 0:
            return

        goal_pose = msg.poses[-1].pose

        x = goal_pose.position.x
        y = goal_pose.position.y

        q = goal_pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z),
            1.0 - 2.0 * (q.z * q.z)
        )

        rounded = (round(x, 2), round(y, 2))

        if self.last_saved == rounded:
            return

        self.new_goal = (x, y, yaw)

    def process_goal(self):

        if self.new_goal is None:
            return

        x, y, yaw = self.new_goal
        self.new_goal = None

        location_name = input("\nEnter location name: ")

        self.save_to_yaml(location_name, x, y, yaw)
        self.last_saved = (round(x, 2), round(y, 2))

    def save_to_yaml(self, name, x, y, yaw):

        if os.path.exists(self.yaml_file):
            with open(self.yaml_file, 'r') as file:
                data = yaml.safe_load(file) or {}
        else:
            data = {}

        data[name] = {
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw)
        }

        with open(self.yaml_file, 'w') as file:
            yaml.dump(data, file, default_flow_style=False)

        self.get_logger().info(
            f"Saved '{name}': x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Nav2PlanSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
