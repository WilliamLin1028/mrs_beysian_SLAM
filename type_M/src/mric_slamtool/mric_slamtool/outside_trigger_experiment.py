import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

# from mric_slamtool_interfaces.srv import StartExperiment

import os
import time
import pandas as pd
import yaml

robot_name = 'robot_D'

class ExperimentTrigger(Node):
    def __init__(self):
        super().__init__(f'{robot_name}_experiment_trigger_node')
        
        # self.srv = self.create_service(StartExperiment, f'{robot_name}_experiment_trigger', self.node_action)
        
        self.sub_experiment_start = self.create_subscription(String, '/start_experiment', self.start_experiment_callback, 10)
        self.experiment_status_msg = None
        self.count = 0
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.node_action)
        
        bringup_dir = get_package_share_directory('mric_slamtool')
        self.cmd_start_experiment_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_start_experiment.txt')
        self.cmd_attend_robot_path = os.path.join(bringup_dir, 'cmd_folder')
        
    def start_experiment_callback(self, msg):
        self.experiment_status_msg = msg.data


    def node_action(self):
        if self.experiment_status_msg == None:
            if self.count == 0:
                self.get_logger().info('Waiting for Experiment Start ...')
                self.count += 1
        else:
            if not os.path.exists(self.cmd_start_experiment_path):
                attend_robot_list =[]
                msg, attend_robot_str = self.experiment_status_msg.split(';')
                self.get_logger().info('Recive topic message. {}'.format(msg))
                attend_robot_list = attend_robot_str.split(',')
                
                for i in range(len(attend_robot_list)):
                    self.get_logger().info('{} : Attend'.format(attend_robot_list[i]))
                    path = os.path.join(self.cmd_attend_robot_path, 'cmd_{}_attend.txt'.format(attend_robot_list[i]))
                    open(path, 'w').close()
                    
                open(self.cmd_start_experiment_path, 'w').close()
        

def main():
    rclpy.init()
    agent = ExperimentTrigger()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()