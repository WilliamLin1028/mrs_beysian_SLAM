import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String 
from ament_index_python.packages import get_package_share_directory

from mric_slamtool_interfaces.srv import ConnectReciver, UpdateTaskList, AllocateTask, CheckAuctioneer

import os
import time
import random
import yaml
import numpy as np
import sys


robot_name = 'robot_A'

class CheckAuctioneerAgent(Node):
    def __init__(self):
        super().__init__(f'{robot_name}_check_auctioneer_node')
        bringup_dir = get_package_share_directory('mric_slamtool')

        # 用來確認是否有其他機器人正在運行
        self.auctioneerlocker_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_wait_for_auctioneer.txt')
        # 確認參與機器人指令檔案的資料夾
        self.attend_cmd_file_path = os.path.join(bringup_dir, 'cmd_folder')
        
        self.order_file = os.path.join(bringup_dir, 'config', 'host_priority.yaml')

        self.cli_requrst = self.create_service(CheckAuctioneer, f'{robot_name}_check_auctioneer', self.node_action)
      
        
        # # 用來向其他機器人發送 auctioneer_lock 資訊
        # self.cli_to_rbA = self.create_client(ConnectReciver, 'robot_A_reciver')
        # self.cli_to_rbB = self.create_client(ConnectReciver, 'robot_B_reciver')
        # self.cli_to_rbC = self.create_client(ConnectReciver, 'robot_C_reciver')
        # self.cli_to_rbD = self.create_client(ConnectReciver, 'robot_D_reciver')
        # self.cli_to_rbE = self.create_client(ConnectReciver, 'robot_E_reciver')
        # self.req = ConnectReciver.Request()
        
        # 監聽其他機器人的工作狀態
        self.sub_robot_A_workstatus = self.create_subscription(String, '/robot_A_workstatus', self.robot_A_workstatus_callback, 10)
        self.sub_robot_B_workstatus = self.create_subscription(String, '/robot_B_workstatus', self.robot_B_workstatus_callback, 10)
        self.sub_robot_C_workstatus = self.create_subscription(String, '/robot_C_workstatus', self.robot_C_workstatus_callback, 10)
        self.sub_robot_D_workstatus = self.create_subscription(String, '/robot_D_workstatus', self.robot_D_workstatus_callback, 10)
        self.sub_robot_E_workstatus = self.create_subscription(String, '/robot_E_workstatus', self.robot_E_workstatus_callback, 10)
        self.robot_A_workstatus = None
        self.robot_B_workstatus = None
        self.robot_C_workstatus = None
        self.robot_D_workstatus = None
        self.robot_E_workstatus = None
        
        self.order_dict = None

        self.attend_robot = []


   
    # workstatus 只會有 ongoing 與 standby 兩個資訊
    def robot_A_workstatus_callback(self, msg):
        self.robot_A_workstatus = msg.data
        
    def robot_B_workstatus_callback(self, msg):
        self.robot_B_workstatus = msg.data
        
    def robot_C_workstatus_callback(self, msg):
        self.robot_C_workstatus = msg.data
        
    def robot_D_workstatus_callback(self, msg):
        self.robot_D_workstatus = msg.data
        
    def robot_E_workstatus_callback(self, msg):
        self.robot_E_workstatus = msg.data
    

    def check_attend_robot(self):
        # 輸出的 self.attend_robot 格式為 ['robot_B', 'robot_D']
        robot_name_list = ['robot_A', 'robot_B', 'robot_C', 'robot_D', 'robot_E']

        for name in robot_name_list:
            file = os.path.join(self.attend_cmd_file_path, f'cmd_{name}_attend.txt')
            if os.path.exists(file):
                self.attend_robot.append(name)
                    
        self.get_logger().info('Attend robots: {}'.format(self.attend_robot))


    def read_order_info(self, file_path):
        # 確認檔案是否已被開啟的假檔案
        lock_file = file_path + '.lock'
        
        if not os.path.exists(lock_file):
            f = open(file_path, 'r', encoding='utf-8')
            order_info = yaml.load(f.read(), Loader=yaml.FullLoader)
            f.close()

            for key, value in order_info.items():
                if key == 'robot_A':
                    A_order = value
                elif key == 'robot_B':
                    B_order = value
                elif key == 'robot_C':
                    C_order = value
                elif key == 'robot_D':
                    D_order = value
                elif key == 'robot_E':
                    E_order = value
                    
                if os.path.exists(lock_file):
                    os.remove(lock_file)

            order_dict = {
                        'robot_A': A_order,
                        'robot_B': B_order,
                        'robot_C': C_order,
                        'robot_D': D_order,
                        'robot_E': E_order
                    }

            return order_dict

        
    def get_auctioneer_robot(self, order_dict, attend_robot):
        """
        從 order_dict 中提取參與實驗的機器人，並取出數值最少的機器人名稱
        """
        # p.s.此方法在出現無法聯繫的機器人時，依然會分配任務給他
        work_status_dict = {
                'robot_A': self.robot_A_workstatus,
                'robot_B': self.robot_B_workstatus,
                'robot_C': self.robot_C_workstatus,
                'robot_D': self.robot_D_workstatus,
                'robot_E': self.robot_E_workstatus
            }

        self.get_logger().info('A: {}'.format(self.attend_robot))
        self.get_logger().info('B: {}'.format(work_status_dict))
        # 只提取參與實驗且狀態不是 ongoing 的機器人
        result = {k: v for k, v in order_dict.items() if k in attend_robot and work_status_dict.get(k) == 'standby'}
        # 如果有符合條件的機器人，找出數值最小的機器人
        if result:
            min_robot = min(result, key=result.get)
            self.get_logger().info('{} is the auctioneer robot.'.format(min_robot))
        
        else:
            self.get_logger().info("There are no matching robots")
        
        if not result:
            return None
        
        return min_robot


    def start_and_finish_lock_allocation(self, act):
        # act 只有 Create 與 Remove 兩種字串
        # 此階段用來確認那些機器人需要傳送指令(即參與實驗的機器人)
        robot_name_list = ['robot_A', 'robot_B', 'robot_C', 'robot_D', 'robot_E']
        
        # 如果參與實驗的機器人在 robot_name_list 中, 則取得該機器人在 list 中的排序, 供 send_request 選擇其對應的傳送函數
        # 回傳的內容為 future
        for robot in self.attend_robot:
            if robot in robot_name_list:
                p = robot_name_list.index(robot)
                terminal_msg = f'{act} {robot} lock file.'
                lock_msg = f'{act} lock file'
                assigned_msg = ''
                future = self.send_srv_request(p , robot, terminal_msg, lock_msg, assigned_msg)
                response = future.result()
                self.get_logger().info('Recive {} feedback: {}'.format(robot, response.reply_allocation))
        


    def node_action(self, request, response):
        # 當機器人收到 check 請求時才運行
        self.get_logger().info('Recive client msg : {}'.format(request.send_msg))
        
        # 取得機器人 order 與 參與實驗的機器人
        if self.order_dict == None:
            self.get_logger().info(f'Loading auctioneer oder file ...')
            self.order_dict = self.read_order_info(self.order_file)
        
        if self.attend_robot == []:
            self.check_attend_robot()
        
        # 確認當前 "閒置的機器人" 與 "拍賣員順位(host_prioity.yaml)" 後,確認自身是否為拍賣員
        if not os.path.exists(self.auctioneerlocker_file_path):
            auctioneer_robot = self.get_auctioneer_robot(self.order_dict, self.attend_robot)
            self.get_logger().info(f'This robot : {robot_name}')
            self.get_logger().info(f'Auctioneer robot : {auctioneer_robot}')
            
            if auctioneer_robot == robot_name:
                self.get_logger().info('This robot is the auctioneer.')
                response.reply_check_result = 'Yes'
                # 如果是拍賣官的話, 用srv向機器人要求建立 auctioneerlocker
                
            else :
                self.get_logger().info('This robot is Not the auctioneer.')
                self.get_logger().info('Waiting for task allocation.')
                response.reply_check_result = 'No'
                
        else :
                self.get_logger().info('There is already other robot becoming auctioneer.')
                self.get_logger().info('Waiting for task allocation.')
                response.reply_check_result = 'No'
        
        return response


def main():
    rclpy.init()
    agent = CheckAuctioneerAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
