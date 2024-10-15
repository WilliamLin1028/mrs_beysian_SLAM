#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from example_interfaces.srv import Trigger
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from sensor_msgs.msg import Imu
import os
from ament_index_python.packages import get_package_share_directory

from tf2_ros import TransformBroadcaster
import tf_transformations

import numpy as np
import math
import time

pkg_path = os.path.join(get_package_share_directory('ominibot_hv'))
from ominibot_hv.OminiBot_HV import ominibothv

from rclpy.duration import Duration


class Ominibot_Robot(Node):

    def __init__(self):
        super().__init__('ominibot_hv')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odomPublisher = self.create_publisher(Odometry, '/odom', 10)
        self.imuPublisher = self.create_publisher(Imu, '/imu', 10)
        self.batteryPublisher = self.create_publisher(Float32, '/battery', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.srv = self.create_service(Trigger, 'drive_square_path', self.drive_square_path)

        self.device_com_port = '/dev/ominibot'
        
        try:
            self.robot_control = ominibothv(
                port = '/dev/ominibot',
                baud = 115200,
                divisor_mode = 3,
                motor_direct = 2,
                encoder_direct = 0,
                motor_pwm_max = 5200,
                motor_pwm_min = 720,
                encoder_ppr = 390,
                wheel_space = 120,
                axle_space = 0,
                gear_ratio = 30,
                wheel_diameter = 58,
                pos_kp = 3000,
                pos_ki = 1050,
                pos_kd = 0,
                vel_kp = 3000,
                vel_ki = 1050)
        except:
            self.get_logger().info('No such directory: "%s"' % self.device_com_port)
            
            while True:
                pass
        
        self.get_logger().info('robot connected from com port: "%s"' % self.device_com_port)
        
        self.robot_control.robot_speed(0, 0, 0)
        self.x = 0.0  # 機器人的x坐標
        self.y = 0.0  # 機器人的y坐標
        self.theta = 0.0  # 機器人的姿態
        self.linear_x_vel, self.linear_y_vel, self.angular_z_vel = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()
        self.battry = 0.0
        self.battry_level = 10.8
        
        try:
            checkCmd, robot_vel, imu_val, bat_val = self.robot_control.read_robot_data()
            self.battry = int.from_bytes(bat_val, byteorder='big', signed=True)/1000
            
            self.get_logger().info("robot battry: {}v".format(self.battry))
            
        except:
            self.get_logger().info("robot battry: can't read .")
        
        if self.battry < self.battry_level:
            while True:
                self.get_logger().info("=== Warning ===")
                self.get_logger().info("robot battry: {}v".format(self.battry))
                self.get_logger().info("The operation stops when the battery is too low. Please turn off the power and charge.")
                time.sleep(5)
        
    
    def cmd_vel_callback(self, msg):
        self.linear_x_vel = msg.linear.x
        self.linear_y_vel = msg.linear.y
        self.angular_z_vel = msg.angular.z

    def send_robot_vel(self):
        self.robot_control.robot_speed(self.linear_x_vel, self.linear_y_vel, self.angular_z_vel)

    def update_robot_state(self):
        # read robot state value
        checkCmd, robot_vel, imu_val, bat_val = self.robot_control.read_robot_data()
        if checkCmd == 1:
            
            self.battry = int.from_bytes(bat_val, byteorder='big', signed=True)/1000
            if self.battry < self.battry_level:
                while True:
                    self.robot_control.robot_speed(0.0, 0.0, 0.0)
                    self.get_logger().info("=== Warning ===")
                    self.get_logger().info("robot battry: {}v".format(self.battry))
                    self.get_logger().info("The operation stops when the battery is too low. Please turn off the power and charge.")
                    time.sleep(5)
            
            lx = int.from_bytes(robot_vel[:2], byteorder='big', signed=True)/1000
            ly = int.from_bytes(robot_vel[2:4], byteorder='big', signed=True)/1000
            az = int.from_bytes(robot_vel[4:], byteorder='big', signed=True)/1000
            
            imu_vx = int.from_bytes(imu_val[:2], byteorder='big', signed=True)/1000
            imu_vy = int.from_bytes(imu_val[2:4], byteorder='big', signed=True)/1000
            imu_vz = int.from_bytes(imu_val[4:6], byteorder='big', signed=True)/1000
            
            imu_ax = int.from_bytes(imu_val[6:8], byteorder='big', signed=True)/1000
            imu_ay = int.from_bytes(imu_val[8:10], byteorder='big', signed=True)/1000
            imu_az = int.from_bytes(imu_val[10:12], byteorder='big', signed=True)/1000
            
            imu_qw = int.from_bytes(imu_val[12:14], byteorder='big', signed=True)/1000
            imu_qx = int.from_bytes(imu_val[14:16], byteorder='big', signed=True)/1000
            imu_qy = int.from_bytes(imu_val[16:18], byteorder='big', signed=True)/1000
            imu_qz = int.from_bytes(imu_val[18:], byteorder='big', signed=True)/1000
            
            
            # self.get_logger().info('x:{}, y:{}, z:{}'.format(lx, ly, az))
             
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time
            
            delta_lx = (lx * math.cos(self.theta) - ly * math.sin(self.theta)) *dt
            delta_ly = (lx * math.sin(self.theta) + ly * math.cos(self.theta)) *dt
            delta_az = az * dt
            
            self.x += delta_lx
            self.y += delta_ly
            self.theta += delta_az

            q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)
            
            
            # 發布機器人odom的轉換
            
            transform_msg = TransformStamped()
            transform_msg.header.stamp = current_time.to_msg()
            transform_msg.header.frame_id = "odom"
            transform_msg.child_frame_id = "base_footprint"
            transform_msg.transform.translation.x = self.x
            transform_msg.transform.translation.y = self.y
            transform_msg.transform.translation.z = 0.0
            
            transform_msg.transform.rotation.x = q[0]
            transform_msg.transform.rotation.y = q[1]
            transform_msg.transform.rotation.z = q[2]
            transform_msg.transform.rotation.w = q[3]
            
            self.tf_broadcaster.sendTransform(transform_msg)
            
            
            
            # 發布機器人里程計
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            odom.header.stamp = current_time.to_msg()
        
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            
            odom.twist.twist.linear.x = lx
            odom.twist.twist.linear.y = ly
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = az

            self.odomPublisher.publish(odom)
            
            
            # 發布機器人IMU
            imu = Imu()
            imu.header.frame_id = "imu_link"
            imu.header.stamp = current_time.to_msg()

            imu.orientation.x = imu_qx
            imu.orientation.y = imu_qy
            imu.orientation.z = imu_qz
            imu.orientation.w = imu_qw

            imu.angular_velocity.x = imu_ax
            imu.angular_velocity.y = imu_ay
            imu.angular_velocity.z = imu_az
            
            imu.linear_acceleration.x = imu_vx
            imu.linear_acceleration.y = imu_vy
            imu.linear_acceleration.z = imu_vz
            
            self.imuPublisher.publish(imu)
            
            battery_msg = Float32()
            battery_msg.data = int.from_bytes(bat_val, byteorder='big', signed=True)/1000
            
            self.batteryPublisher.publish(battery_msg)
        else:
            self.get_logger().info("error: {}".format("lost some messages."))
    
    def drive_square_path(self, request, response):

        self.get_logger().info('Run the robot service.')
        
        # x+
        self.robot_control.robot_speed(0.1, 0, 0)
        time.sleep(3)
        
        # y-
        self.robot_control.robot_speed(0, -0.1, 0)
        time.sleep(3)

        # x-
        self.robot_control.robot_speed(-0.1, 0, 0)
        time.sleep(3)
        
        # y+
        self.robot_control.robot_speed(0, 0.1, 0)
        time.sleep(3)

        self.robot_control.robot_speed(0, 0, 0)

        self.get_logger().info('Success.')
        
        response.success = True
        response.message = "Stop"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = Ominibot_Robot()
    node.last_time = node.get_clock().now()
    timer_update = 0.02  # 定時器週期
    timer_send = 0.1   # 定時器週期
    node.create_timer(timer_update, node.update_robot_state)
    node.create_timer(timer_send, node.send_robot_vel)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

