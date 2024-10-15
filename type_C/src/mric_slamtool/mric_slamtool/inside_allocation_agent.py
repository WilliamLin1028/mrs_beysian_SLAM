import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator

from rclpy.action import ActionClient

from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String 

# from mric_action.action import  GetSuccessrate
from ament_index_python.packages import get_package_share_directory

import os
import time
import random
import numpy as np
from scipy.optimize import linear_sum_assignment
import pandas as pd

class AllocationAgent(Node):
    def __init__(self):
        super().__init__('allocation_agent_node')

        # self._action_client = ActionClient(self, GetSuccessrate, 'get_successrate')
        
        self.bringup_dir = get_package_share_directory('mric_slamtool')
        self.cmd_allocation = os.path.join(self.bringup_dir, 'cmd_folder', 'cmd_allocation.txt')
        self.experiment_config_file_path = os.path.join(self.bringup_dir, 'config', 'experiment_config.txt')
        self.task_allocation_path = os.path.join(self.bringup_dir, 'database', 'best_assignment.txt')
        self.task_array_txt = os.path.join(self.bringup_dir, 'database', 'task_array.txt')
        
        self.experiment_config = None
        
        
    def check_file_existence(self, filepath):
        """
        檢查指定目錄內是否存在指定檔案
        """
        
        # 檢查檔案是否存在
        if os.path.isfile(filepath):
            return 'file is exist'
        else:
            return 'file is not exist'    
    
    def read_experiment_config(self, file_path):
        """
        讀取實驗配置文件
        """
        config = {}
        # 確認檔案是否已被開啟的假檔案
        lock_file = file_path + '.lock'
        if not os.path.exists(lock_file):
            open(lock_file, 'w').close()
            with open(file_path, 'r') as file:
                lines = file.readlines()
                for line in lines:
                    if ':' in line:
                        robot_name, status = line.strip().split(':')
                        config[robot_name.strip()] = status.strip()
                        
            # 刪除假檔案標示檔案已關閉
            if os.path.exists(lock_file):
                os.remove(lock_file)
                
        return config
    
    def read_success_rate(file_path):
        """
        讀取機器人的成功率文件
        """
        success_rates = {}
        with open(file_path, 'r') as file:
            content = file.read().strip()
            tasks = content.split(';')
            for task in tasks:
                if ':' in task:
                    task_name, rate = task.split(':')
                    success_rates[task_name.strip()] = float(rate.strip())
        return success_rates
    
    # 函數：顯示格式化的成功率表格
    def display_success_rate_table(self, success_rates, assignment):
        # 創建一個 DataFrame
        df = pd.DataFrame(success_rates).T
        
        # 四捨五入成功率到小數點後5位
        df = df.round(5)
        
        # 標記最佳配對
        for robot, task in assignment.items():
            df.at[robot, task] = '*' + str(df.at[robot, task])
        
        # 打印表格
        self.get_logger().info(df)
    
    def save_assignment_to_file(self, assignment):
        # 格式化配對結果為字串
        assignment_str = ';'.join([f'{robot}:{task}' for robot, task in assignment.items()])
        
        # 檢查檔案是否存在，如果存在則刪除
        if os.path.exists(self.task_allocation_path):
            self.get_logger().info('File is exist. Remove old version.')
            os.remove(self.task_allocation_path)
        
        # 將配對結果寫入檔案
        with open(self.task_allocation_path, 'w') as file:
            file.write(assignment_str)
            self.get_logger().info('best_assignment.txt updated.')
    
    def load_taskarray(self):
        self.get_logger().info('[load_taskarray] ---- start.')
        self.get_logger().info('Start loading task array file.')
        
        # 確認檔案是否已被開啟的假檔案
        lock_file = self.task_array_txt + '.lock'

        # 等待直到檔案被釋放
        if os.path.exists(lock_file):
            self.get_logger().info('Task array file is currently in use, waiting...')
            time.sleep(0.1)  # 等待0.1秒後再檢查
        else:
            # 只在假檔案不存在時才執行
            # 創建假檔案標示檔案被開啟
            open(lock_file, 'w').close() 
            
            with open(self.task_array_txt, 'r') as file:
                task_string = file.read().strip()
                
            """
            解析任務字串並將其存入 dictionary 中
            """
            # 初始化空的 dictionary
            tasks_dict = {}

            # 用 ';' 分隔不同的任務
            tasks = task_string.split(';')

            # 遍歷每個任務
            for task in tasks:
                if ':' in task and ',' in task:
                    # 用 ':' 分隔任務名稱和任務資訊
                    task_name, task_info = task.split(':')
                    
                    # 用 ',' 分隔任務資訊的細項
                    task_status, x_pose, y_pose = task_info.split(',')
                    
                    # 將資訊存入 dictionary
                    tasks_dict[task_name] = {
                        'status': task_status,
                        'x_pose': float(x_pose),
                        'y_pose': float(y_pose),
                        'cost': None
                    }
            
            # 刪除假檔案標示檔案已關閉
            if os.path.exists(lock_file):
                os.remove(lock_file)
        
        # self.get_logger().info('Task array loading completed.')
        self.get_logger().info('[load_taskarray] ---- completion.')
        
        return tasks_dict
    
    def task_allocation_function(self):
        # 讀取機器人配置文件
        if self.experiment_config == None:
            file_check = self.check_file_existence(self.experiment_config_file_path)
            if file_check == 'file is not exist':
                self.get_logger().info('Experiment config file is not exist.')
            else:
                self.get_logger().info('File is exist.')
                self.experiment_config = self.read_experiment_config(self.experiment_config_file_path)
        
        if os.path.isfile(self.cmd_allocation) & self.experiment_config != None :
            
            # 初始化任務成功率矩陣
            success_rate_matrix = {}
            
             # 收集需要的成功率文件列表
            required_files = []

            # 根據配置文件決定是否讀取對應機器人的成功率文件
            for robot_name, status in self.experiment_config.items():
                if status == 'Attend':
                    robot_id = robot_name.split('_')[1]  # 提取機器人ID (A, B, C, ...)
                    success_rate_file_path = os.path.join(self.bringup_dir, 
                                                          'database', 
                                                          f'robot_{robot_id}_successrate_array.txt')
                    required_files.append(success_rate_file_path)

            # 檢查所有需要的成功率文件是否存在
            missing_files = [file for file in required_files if not os.path.exists(file)]

            if missing_files:
                self.get_logger().info("The following success rate files are missing:")
                for file in missing_files:
                    self.get_logger().info(file)
                return

            
            # 根據配置文件決定是否讀取對應機器人的成功率文件
            self.get_logger().info('Loading task success rate...')
            for robot_name, status in self.experiment_config.items():
                if status == 'Attend':
                    robot_id = robot_name.split('_')[1]  # 提取機器人ID (A, B, C, ...)
                    self.get_logger().info('Read robot {} task success rate.'.format(robot_id))
                    success_rate_file_path = os.path.join(self.bringup_dir, 'database', f'robot_{robot_id}_successrate_array.txt')
                    # success_rate_file_path = f'robot_{robot_id}_successrate_array.txt'
                    success_rates = self.read_success_rate(success_rate_file_path)
                    success_rate_matrix[robot_name] = success_rates

            # 輸出任務成功率矩陣
            for robot_name, success_rates in success_rate_matrix.items():
                self.get_logger().info(f"Robot: {robot_name}")
                for task_name, rate in success_rates.items():
                    self.get_logger().info(f"  {task_name}: {rate}")
                    
            # 取得機器人和任務的列表
            robots = list(success_rate_matrix.keys())
            tasks = list(success_rate_matrix[robots[0]].keys())

            # 構建成本矩陣 (取負的成功率)
            self.get_logger().info('Build cost matrix...')
            cost_matrix = np.array([[-success_rate_matrix[robot][task] for task in tasks] for robot in robots])

            ### 使用匈牙利演算法求解最佳分配(全局最高解)
            self.get_logger().info('Find optimal allocation using Hungarian algorithm...')
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            
            # 顯示結果
            assignment = {}
            for r, c in zip(row_ind, col_ind):
                assignment[robots[r]] = tasks[c]

            self.get_logger().info("Best combination of tasks:")
            for robot, task in assignment.items():
                self.get_logger().info(f"{robot} -> {task}")
            
            # 顯示成功率表格
            self.display_success_rate_table(success_rates, assignment)
            self.save_assignment_to_file(assignment)
        
        return
        


def main(args=None):
    rclpy.init(args=args)
    agent = AllocationAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
