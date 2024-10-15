import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String 
from ament_index_python.packages import get_package_share_directory

from mric_slamtool_interfaces.srv import ConnectReciver, UpdateTaskList, AllocateTask, CheckAuctioneer, LockAuctioneer

import os
import time
import random
import yaml
import numpy as np
import sys


robot_name = 'robot_D'

class TaskAgent(Node):
    def __init__(self):
        super().__init__(f'{robot_name}_auctioneer_agent_main_node')
        self.sub_node = rclpy.create_node(f'{robot_name}_auctioneer_agent_sub_node')
        
        bringup_dir = get_package_share_directory('mric_slamtool')
        # 用來確認機器人是否待機
        self.robot_standby_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_robot_standby.txt')
        self.robot_ongoing_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_robot_ongoing.txt')
        # 用來確認實驗是否開始
        self.experiment_start_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_start_experiment.txt')
        # 用來確認是否有其他機器人正在運行
        self.auctioneerlocker_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_wait_for_auctioneer.txt')
        # 確認參與機器人指令檔案的資料夾
        self.attend_cmd_file_path = os.path.join(bringup_dir, 'cmd_folder')
        # 建立用於觸發 outside_requester_toA~E.py 要求 successrate 的檔案
        self.requester_trigger_file = os.path.join(bringup_dir, 'cmd_folder', 'cmd_request_successrate.txt')
        # 建立用於觸發 inside_allocation_agent.py 用貝氏網路計算 best_assignment.txt 的檔案
        self.allocation_trigger_file = os.path.join(bringup_dir, 'cmd_folder', 'cmd_allocation.txt')
        
        
        self.order_file = os.path.join(bringup_dir, 'config', 'host_priority.yaml')
        
        self.task_list_file_path = os.path.join(bringup_dir, 'database', 'task_list.txt')
        self.ongoing_task_file_path = os.path.join(bringup_dir, 'database', 'ongoing_task.txt')
        self.goal_fail_file = os.path.join(bringup_dir, 'database', 'goal_fail.txt')
        
        self.best_assignment_path = os.path.join(bringup_dir, 'database', 'best_assignment.txt')
        self.database_path = os.path.join(bringup_dir, 'database')
        
        # 確認自身是否為 auctioneer
        self.cli_check_auctioneer = self.create_client(CheckAuctioneer, f'{robot_name}_check_auctioneer')
        while not self.cli_check_auctioneer.wait_for_service(timeout_sec=0.2):
            self.get_logger().info('Check Auctioneer Agent service not available, waiting again...')
        self.req_check_auctioneer = CheckAuctioneer.Request()
        
        
        self.cli_for_update_task = self.create_client(UpdateTaskList, f'{robot_name}_update_task')
        while not self.cli_for_update_task.wait_for_service(timeout_sec=0.2):
            self.get_logger().info('Task Agent service not available, waiting again...')
        self.req_update_task = UpdateTaskList.Request()
        
        
        self.cli_for_task_allocation = self.create_client(AllocateTask, f'{robot_name}_allocation_agent')
        while not self.cli_for_task_allocation.wait_for_service(timeout_sec=0.2):
            self.get_logger().info('Allocation Agent service not available, waiting again...')
        self.req_task_allocation = AllocateTask.Request()
        
        # auctioneer lock 
        self.cli_for_rbA_lock = self.create_client(LockAuctioneer, 'robot_A_lock_reciver')
        self.cli_for_rbB_lock = self.create_client(LockAuctioneer, 'robot_B_lock_reciver')
        self.cli_for_rbC_lock = self.create_client(LockAuctioneer, 'robot_C_lock_reciver')
        self.cli_for_rbD_lock = self.create_client(LockAuctioneer, 'robot_D_lock_reciver')
        self.cli_for_rbE_lock = self.create_client(LockAuctioneer, 'robot_E_lock_reciver')
        self.req_lock = LockAuctioneer.Request()
        
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.main_node_action)
        
        self.work_status_change = None
        self.experiment_status_change = None
        
        self.explore_space = [2.0, 3.0, -2.0, -2.0]     # 探索範圍 [x_max, y_max, x_min, y_min] 
        self.task_num = 13
        # self.attend_robot = ['robot_B', 'robot_D']
        self.attend_robot = []
        
        self.count = 0


    def check_file_existence(self, file_path):
        """
        參數:   directory (str): 要檢查的資料夾路徑
                filename (str): 要檢查的檔案名稱
        回傳:   str: 檔案是否存在的訊息
        """
        if os.path.isfile(file_path):
            return True
        else:
            return False
    

    def check_attend_robot(self):
        # 輸出的 self.attend_robot 格式為 ['robot_B', 'robot_D']
        robot_name_list = ['robot_A', 'robot_B', 'robot_C', 'robot_D', 'robot_E']
        if self.attend_robot == []:
            for name in robot_name_list:
                file = os.path.join(self.attend_cmd_file_path, f'cmd_{name}_attend.txt')
                if os.path.exists(file):
                    self.attend_robot.append(name)
                    
            self.get_logger().info('Attend robots: {}'.format(self.attend_robot))
    
        
    def start_and_finish_lock_allocation(self, act):
        # act 只有 Create 與 Remove 兩種字串
        # 此階段用來確認那些機器人需要傳送指令(即參與實驗的機器人)
        robot_name_list = ['robot_A', 'robot_B', 'robot_C', 'robot_D', 'robot_E']
        
        # 如果參與實驗的機器人在 robot_name_list 中, 則取得該機器人在 list 中的排序, 供 send_request 選擇其對應的傳送函數
        # 回傳的內容為 future
        for robot in self.attend_robot:
            if (robot in robot_name_list) and (robot != robot_name):
                p = robot_name_list.index(robot)
                terminal_msg = f'{act} {robot} lock file.'
                lock_msg = f'{act} lock file'
                future = self.send_srv_request(p, terminal_msg, lock_msg, robot)
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                self.get_logger().info('Recive {} feedback: {}'.format(robot, response.reply_lock))
        
    def send_srv_request(self, p, terminal_msg, lock_msg, rb):
        cli_list = [self.cli_for_rbA_lock, 
                    self.cli_for_rbB_lock, 
                    self.cli_for_rbC_lock, 
                    self.cli_for_rbD_lock, 
                    self.cli_for_rbE_lock] 
        
        while not cli_list[p].wait_for_service(timeout_sec=0.2):
            self.get_logger().info(f'{rb} ac_lock service not available, waiting again...')
            
        self.get_logger().info(terminal_msg)
        self.req_lock.allocation_lock = lock_msg
        return cli_list[p].call_async(self.req_lock)
            
    def send_update_tasklist_req(self, pub_msg):
        # while not self.cli_for_update_task.wait_for_service(timeout_sec=0.2):
        #     self.get_logger().info('service not available, waiting again...')
            
        self.get_logger().info(f'Request {robot_name}_task_agent update task list...')
        
        self.req_update_task.send_msg = pub_msg
        
        return self.cli_for_update_task.call_async(self.req_update_task)
    
    # 確認機器人是否閒置且實驗是否開始
    def node_action_phase_1(self):

        # 確認機器是否進入待機(standby)
        standby_message = self.check_file_existence(self.robot_standby_file_path)
        if self.work_status_change != standby_message:
            self.work_status_change = standby_message
            self.get_logger().info(f'Standby_message : {self.work_status_change}')
            
        # check 是否開始實驗
        experiment_start_message = self.check_file_existence(self.experiment_start_file_path)
        if self.experiment_status_change != experiment_start_message:
            self.experiment_status_change = experiment_start_message
            self.get_logger().info(f'Experiment_start : {self.experiment_status_change}')
            
            if self.count == 0 and self.experiment_status_change == False:
                self.get_logger().info('Wait for Commander start experiment.')
                self.count += 1
            
            # 實驗開始後才確認參與實驗的機器人有哪些
            elif self.attend_robot == [] and self.experiment_status_change == True:
                self.get_logger().info('Experiment is Start.')
                self.get_logger().info('Check attend experiment robots ...')
                self.check_attend_robot()
        
        if (standby_message == True) and (experiment_start_message == True):
            return True
        else:
            return False
        
    # 確認機器人是否為拍賣官，如果是則要求其他機器人不得更新任務
    def node_action_phase_2(self):
        self.req_check_auctioneer.send_msg = 'Check this is auctioneer or not.'
        self.get_logger().info('Send msg to check_auctioneer agent')
        return self.cli_check_auctioneer.call_async(self.req_check_auctioneer)  

    # 取得新的任務 list
    def node_action_phase_3(self, request_msg):
        self.get_logger().info(f'Request {robot_name}_task_agent update task list ...')
        self.get_logger().info(f'Send msg : {request_msg}')
        self.req_update_task.send_msg = request_msg
        return self.cli_for_update_task.call_async(self.req_update_task)

    # 傳送 task list 要求進行任務分配
    def node_action_phase_4(self, request_msg):
        self.get_logger().info(f'Request {robot_name}_allocation_agent assiged task ...')
        self.get_logger().info(f'Send msg : {request_msg}')
        self.req_task_allocation.task_list = request_msg
        return self.cli_for_task_allocation.call_async(self.req_task_allocation)

'''   
    def main_node_action(self):
        ph1 = self.node_action_phase_1()
        if ph1 == True:
            self.get_logger().info('Experiment started.')
            future_auctioneer_check = self.node_action_phase_2()
            rclpy.spin_until_future_complete(self, future_auctioneer_check)
            
            response_1 = future_auctioneer_check.result()
            ph2 = response_1.reply_check_result
            self.get_logger().info('Auctioneer check : {}'.format(ph2))
            
            if ph2 == 'Yes':
                # ph 3
                self.start_and_finish_lock_allocation('Create')
                # 請求更新任務
                self.get_logger().info('Request new task list')
                time.sleep(1)
                future_1 = self.node_action_phase_3('Update task list.')
                rclpy.spin_until_future_complete(self, future_1)
                new_task_response = future_1.result()
                task_list = new_task_response.new_task_list
                # 格式 : {task_name}:{task_status},{x_pose},{y_pose};{task_name}:...
                self.get_logger().info('\nNew Task List :\n{}'.format(task_list))
                
                # 請求任務分配
                self.get_logger().info('Request task allocation')
                future_2 = self.node_action_phase_4(task_list)
                rclpy.spin_until_future_complete(self, future_2)
                best_assignment_response = future_2.result()
                # 格式 : 
                self.get_logger().info('\nAssigment :\n{}'.format(best_assignment_response.best_assignment))
                
                self.get_logger().info('New Task has been assignment. Cancel auctioneer lock .')
                self.start_and_finish_lock_allocation('Remove')
                self.get_logger().info('Auctioneer process is finish.')
'''

def main(args=None):
    rclpy.init(args=args)
    # agent = TaskAgent()
    # rclpy.spin(agent)
    # agent.destroy_node()
    # rclpy.shutdown()
    
    while 1:
        # rclpy.init(args=args)
        agent = TaskAgent()

        ph1 = agent.node_action_phase_1()
        if ph1 == True:
            agent.get_logger().info('Experiment started.')
            future_auctioneer_check = agent.node_action_phase_2()
            rclpy.spin_until_future_complete(agent, future_auctioneer_check)
            response_1 = future_auctioneer_check.result()
            ph2 = response_1.reply_check_result
            agent.get_logger().info('Auctioneer check : {}'.format(ph2))
            
            if ph2 == 'Yes':
                agent.start_and_finish_lock_allocation('Create')
                # 請求更新任務
                agent.get_logger().info('request new task list')
                future_1 = agent.node_action_phase_3('Update task list.')
                rclpy.spin_until_future_complete(agent, future_1)
                new_task_response = future_1.result()
                task_list = new_task_response.new_task_list
                # 格式 : {task_name}:{task_status},{x_pose},{y_pose};{task_name}:...
                agent.get_logger().info('New Task List :\n{}'.format(task_list))
                
                # 請求任務分配
                agent.get_logger().info('request task allocation')
                future_2 = agent.node_action_phase_4(task_list)
                rclpy.spin_until_future_complete(agent, future_2)
                best_assignment_response = future_2.result()
                # 格式 : 
                agent.get_logger().info('Assigment :\n{}'.format(best_assignment_response.best_assignment))
                time.sleep(1)
                agent.get_logger().info('New Task has been assignment. Cancel auctioneer lock .')
                agent.start_and_finish_lock_allocation('Remove')
                agent.get_logger().info('Auctioneer process is finish.')

        time.sleep(0.2)
        agent.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
