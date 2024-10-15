import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import yaml
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

'''
這個程式是透過掃描 database 中是否存在 robot_standby.txt 與robot_standby.txt 檔, 來確認要發布的機器人運行狀態
'''
robot_name = 'robot_A'

class PubWorkStatus(Node):
    def __init__(self):
        super().__init__(f'pub_{robot_name}_workstatus_agent')
        bringup_dir = get_package_share_directory('mric_slamtool')
        
        self.robot_standby_file_path = os.path.join(bringup_dir, 'cmd_folder', 'robot_standby.txt')
        self.robot_ongoing_file_path = os.path.join(bringup_dir, 'cmd_folder', 'robot_ongoing.txt')
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.node_action)
        self.pub_workstatus = self.create_publisher(String, f'/{robot_name}_workstatus', 10)
        
    def publish_workstatus_msg(self, msg):
        self.pub_workstatus.publish(msg)
    
    def check_workstatus_file(self):
        standby = os.path.isfile(self.robot_standby_file_path)
        ongoing = os.path.isfile(self.robot_ongoing_file_path)
        if standby == True & ongoing == False :
            #self.get_logger().info('Robot status : standby')
            return 'standby'
        elif standby == False & ongoing == True :
            #self.get_logger().info('Robot status : ongoing')
            return 'ongoing'
        else :
            #self.get_logger().info('Robot status : error')
            return
    
    
    def node_action(self):
        # 先發佈機器人工作狀態
        workstatus_msg = String()
        workstatus_msg.data = self.check_workstatus_file()
        self.publish_workstatus_msg(workstatus_msg)
        
        
            
def main():
    rclpy.init()
    agent = PubWorkStatus()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()