import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math

class WheelchairNavGoal(Node):
    def __init__(self):
        super().__init__('wheelchair_nav_goal')
        # Create an Action Client to communicate with the Nav2 server
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()

        # Initialize the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set X and Y positions
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Convert Yaw (Euler) to Quaternion for ROS 2
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending goal: X={x}, Y={y}, Yaw={yaw} rad')
        
        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server.')
            return

        self.get_logger().info('Goal accepted! Wheelchair is moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # This triggers when the robot either reaches the goal or fails
        status = future.result().status
        self.get_logger().info(f'Navigation finished with status code: {status}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client_node = WheelchairNavGoal()
    
    # --- EDIT YOUR GOAL HERE ---
    # Target: 2.0 meters forward, 1.0 meter left, facing 90 degrees left (1.57 rad)
    # glass frontt
    target_x = 7.2
    target_y = 4.4
    target_yaw = -1.6

    # # kuldeep
    # target_x = 0.12
    # target_y = 5.19
    # target_yaw = -0.026
    
    action_client_node.send_goal(target_x, target_y, target_yaw)
    
    # Spin the node so it can listen for server callbacks
    rclpy.spin(action_client_node)

if __name__ == '__main__':
    main()
