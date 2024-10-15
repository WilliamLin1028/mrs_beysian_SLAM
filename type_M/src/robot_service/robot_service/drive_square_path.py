from example_interfaces.srv import Trigger
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time


class RobotService(Node):

    def __init__(self):
        super().__init__('robot_service')
        self.srv = self.create_service(Trigger, 'drive_square_path', self.drive_square_path)
        self.velPublisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def drive_square_path(self, request, response):

        self.get_logger().info('Run the robot service.')
        
        # x+
        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.velPublisher.publish(twist)
        
        time.sleep(3)
        
        # y-
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = -0.1
        twist.angular.z = 0.0
        self.velPublisher.publish(twist)
        
        time.sleep(3)

        # x-
        twist = Twist()
        twist.linear.x = -0.1
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.velPublisher.publish(twist)
        
        time.sleep(3)
        
        # y+
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.1
        twist.angular.z = 0.0
        self.velPublisher.publish(twist)
        
        time.sleep(3)

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.velPublisher.publish(twist)

        self.get_logger().info('Success.')
        
        response.success = True
        response.message = "Stop"

        return response

        


def main():
    rclpy.init()

    robot_service = RobotService()

    rclpy.spin(robot_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
