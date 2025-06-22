import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
from geometry_msgs.msg import Quaternion
from nav2_msgs.srv import ManageLifecycleNodes  # For pause/resume
from geometry_msgs.msg import Twist  # For stopping the robot
import os

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q

class NavigationClient(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Publisher for stopping the robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Service client for pause/resume
        self.client = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')

        #while not self.client.wait_for_service(timeout_sec=2.0):
            #self.get_logger().warn('Waiting for lifecycle manager service...')

    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw (theta) to quaternion
        q = yaw_to_quaternion(theta)
        goal_msg.pose.pose.orientation = q

        self.get_logger().info(f'Sending goal to x: {x}, y: {y}, theta: {theta}')
        
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by the server')
            # rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted by server, waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')

    def result_callback(self, future):
        result = future.result()
        if result.status == 4:  # Status 4 means success
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().info(f'Goal failed with status: {result.status}')

        # rclpy.shutdown()  # Shutdown after goal completion

    def stop_robot(self):
        """Send zero velocity to stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Robot stopped.')

    def send_request(self, command):
        """Send a PAUSE or RESUME request"""
        request = ManageLifecycleNodes.Request()
        request.command = command
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result() is not None

    def execute(self):
        os.system("xdotool search --onlyvisible --name 'RViz*' windowactivate")
        """Full sequence: Stop → Pause → Resume"""
        self.stop_robot()
        
        self.get_logger().info('Pausing navigation...')
        if self.send_request(ManageLifecycleNodes.Request.PAUSE):
            self.get_logger().info('Navigation paused successfully.')
            
            self.get_logger().info('Resuming navigation...')
            if self.send_request(ManageLifecycleNodes.Request.RESUME):
                self.get_logger().info('Navigation resumed successfully.')
            else:
                self.get_logger().error('Failed to resume navigation.')
        else:
            self.get_logger().error('Failed to pause navigation.')

        # rclpy.shutdown()  # Shutdown after completion
