import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ManageLifecycleNodes  # For pause/resume
from geometry_msgs.msg import Twist  # For stopping the robot

class StopPauseResumeNav(Node):
    def __init__(self):
        super().__init__('stop_pause_resume_nav')
        
        # Publisher for stopping the robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Service client for pause/resume
        self.client = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')

        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Waiting for lifecycle manager service...')

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

# def main():
#     rclpy.init()
#     node = StopPauseResumeNav()
#     node.execute()
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

