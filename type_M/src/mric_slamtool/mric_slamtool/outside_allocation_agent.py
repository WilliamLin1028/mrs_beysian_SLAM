import rclpy
from rclpy.node import Node


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

from mric_slamtool_interfaces.srv import ConnectReciver, UpdateTaskList, AllocateTask, RequestSuccessrate, SendNewTask

robot_name = 'robot_D'

class AllocationAgent(Node):
    def __init__(self):
        super().__init__(f'{robot_name}_allocation_agent_main_node')
        self.sub_node = rclpy.create_node(f'{robot_name}_allocation_agent_sub_node')
        
        # 用 main 來建立 server
        self.srv = self.create_service(AllocateTask, f'{robot_name}_allocation_agent', self.node_action)        

        self.bringup_dir = get_package_share_directory('mric_slamtool')
        
        # 確認參與機器人指令檔案的資料夾
        self.attend_cmd_file_path = os.path.join(self.bringup_dir, 'cmd_folder')
        
        self.task_allocation_path = os.path.join(self.bringup_dir, 'database', 'best_assignment.txt')
        self.task_array_txt = os.path.join(self.bringup_dir, 'database', 'task_array.txt')
        
        # 用 sub_node 來建立 client
        # 建立請求成功率的 client
        self.cli_for_rbA_sr = self.sub_node.create_client(RequestSuccessrate, 'robot_A_successrate_cal')
        self.cli_for_rbB_sr = self.sub_node.create_client(RequestSuccessrate, 'robot_B_successrate_cal')
        self.cli_for_rbC_sr = self.sub_node.create_client(RequestSuccessrate, 'robot_C_successrate_cal')
        self.cli_for_rbD_sr = self.sub_node.create_client(RequestSuccessrate, 'robot_D_successrate_cal')
        self.cli_for_rbE_sr = self.sub_node.create_client(RequestSuccessrate, 'robot_E_successrate_cal')
        self.req_sr = RequestSuccessrate.Request()
        # 建立發送新任務的 client
        self.cli_for_rbA_nt = self.sub_node.create_client(SendNewTask, 'robot_A_newtask')
        self.cli_for_rbB_nt = self.sub_node.create_client(SendNewTask, 'robot_B_newtask')
        self.cli_for_rbC_nt = self.sub_node.create_client(SendNewTask, 'robot_C_newtask')
        self.cli_for_rbD_nt = self.sub_node.create_client(SendNewTask, 'robot_D_newtask')
        self.cli_for_rbE_nt = self.sub_node.create_client(SendNewTask, 'robot_E_newtask')
        self.req_nt = SendNewTask.Request()
        
        
        self.robot_name_list = ['robot_A', 'robot_B', 'robot_C', 'robot_D', 'robot_E']
        self.attend_robot = []
        self.display_num = 5
        
        
    def check_attend_robot(self):
        # 輸出的 self.attend_robot 格式為 ['robot_B', 'robot_D']
        for name in self.robot_name_list:
            file = os.path.join(self.attend_cmd_file_path, f'cmd_{name}_attend.txt')
            if os.path.exists(file):
                self.attend_robot.append(name)
                    
        self.get_logger().info('Attend robots: {}'.format(self.attend_robot))
        
    # 用 server 向其他機器人要求成功率字串並整合在一起
    def requset_task_successrate(self, task_list_a):
        self.get_logger().info('\n ------------------------------ [Start] Requset task successrate ------------------------------')
        self.get_logger().info('Send msg : \n{}'.format(task_list_a))
        srst_list = []
        cli_list = [self.cli_for_rbA_sr, 
                    self.cli_for_rbB_sr, 
                    self.cli_for_rbC_sr, 
                    self.cli_for_rbD_sr, 
                    self.cli_for_rbE_sr] 
        
        # 如果參與實驗的機器人在 self.robot_name_list 中(應該要是 self.robot_name_list 中有參與實驗的機器人)
        # for robot in self.attend_robot:
        #     if robot in self.robot_name_list:
        for robot in self.robot_name_list:
            if robot in self.attend_robot:
                self.get_logger().info(f'{robot} is Attend experiment.')
                p = self.robot_name_list.index(robot)
                
                while not cli_list[p].wait_for_service(timeout_sec=0.2):
                    self.get_logger().info(f'{robot} successrate agent service not available, waiting again...')
                
                self.req_sr.task_list = task_list_a
                future = cli_list[p].call_async(self.req_sr)
                self.get_logger().info(f'Send successrate request to {robot}_successrate_agent. ')
                rclpy.spin_until_future_complete(self.sub_node, future)
                self.get_logger().info(f'Recive {robot}_successrate_agent feedback. ')
                # 成功率字串格式 : task_0:0.2275;task_1:0.1366;task_3:0.5612;task_4:0.3708; .....
                response = future.result()
                srst_list.append(response.successrate_string)
                # self.get_logger().info(f'{robot} task list success rate : \n {response}')
                
            else:
                self.get_logger().info(f'{robot} is Not Attend experiment.')
                srst_list.append(None)
            
            # self.get_logger().info('\n---------- Request callback list ----------')
            # for i in range(len(srst_list)):
            #     self.get_logger().info('{}'.format(srst_list[i]))
            
        self.get_logger().info('\n ------------------------------ Check callback list items ------------------------------')
        # 格式 : srst_list = [rb1_sr, rb2_sr, None, None, None]
        self.get_logger().info('There has {} items in srst_list'.format(len(srst_list)))
        self.get_logger().info('\n ------------------------------ [Finish] Requset task successrate ------------------------------')
        
        return srst_list

    # 將成功率字串裁切成 2維 list , 格式 : [[task_name_0,task_sr_0],[task_name_1,task_sr_1],.......]
    def str2list(self, sr_string):
        list_1 = []
        list_2_t = []
        list_2_s = []
        
        list_1 = sr_string.split(';')
        for i in list_1:
            tk, sr = (i.split(':'))
            list_2_t.append(tk)
            list_2_s.append(float(sr))
            
        # self.get_logger().info('str2list result : {}'.format(list_2))
        
        return list_2_t, list_2_s
    
    # 將成功率字串 list 整理成 total_dict
    def srst_list2dict(self, srst_list):
        # 格式 : srst_list = [rb1_sr, rb2_sr, None, None, None]
        # 需要分別提取並分割
        self.get_logger().info('\n------------------------------ [Start] Transfer string list into dict ------------------------------')
        total_dict = {
            'robot_A': {},
            'robot_B': {},
            'robot_C': {},
            'robot_D': {},
            'robot_E': {}
        }

        for i in range(len(self.robot_name_list)):
            rb_name = self.robot_name_list[i]
            sr_str = srst_list[i]
            
            self.get_logger().info(f'sr_str : {sr_str}')
            sr_list = []
            if sr_str != None:
                tk_list, sr_list = self.str2list(sr_str)
            
            # self.get_logger().info(tk_list)
            # self.get_logger().info(sr_list)
            # Initialize the dictionary for the robot
            total_dict[rb_name] = {}
            
            # Iterate through the list and add tasks and success rates
            if sr_list != []:
                self.get_logger().info('Append {} success rate into total dict, there has {} items in sr_list, {} items in tk_list.'.format(rb_name, len(sr_list), len(tk_list)))
                for i in range(len(sr_list)):
                    self.get_logger().info('{} : {}'.format(tk_list[i], sr_list[i]))
                    total_dict[rb_name][tk_list[i]] = sr_list[i]
                    # total_dict[rb_name][task] = float(success_rate)
            else:
                total_dict[rb_name] = None

        self.get_logger().info(str(total_dict))
        self.get_logger().info('\n------------------------------ [Finish] Transfer string list into dict ------------------------------')
        return total_dict       
        
    def task_assignment(self, total_dict):
        self.get_logger().info('\n------------------------------ [Start] Task allocation ------------------------------')
        # 取得機器人和任務的列表
        robots = list(total_dict.keys())
        self.get_logger().info(str(robots))
        
        # tasks = list(total_dict[robots[0]].keys())
        # self.get_logger().info(str(tasks))
        
        # 過濾掉 total_dict 中值為 None 的機器人
        valid_robots = [robot for robot in robots if total_dict[robot] is not None]

        if not valid_robots:
            self.get_logger().error('No valid robots with tasks found.')
        
        tasks = list(set().union(*[total_dict[robot].keys() for robot in valid_robots]))  

        # 構建成本矩陣 (取負的成功率)
        cost_matrix = np.full((len(valid_robots), len(tasks)), float('inf'))
        for i, robot in enumerate(valid_robots):
            for j, task in enumerate(tasks):
                if task in total_dict[robot]:
                    cost_matrix[i, j] = -total_dict[robot][task]  # 使用負的成功率作為成本

        # 使用匈牙利演算法求解最佳分配，如果有多個機器人
        if len(valid_robots) > 1:
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            optimal_assignment = {valid_robots[i]: tasks[j] for i, j in zip(row_ind, col_ind)}
        else:
            # 只有一個有效機器人，選擇數值最大的任務
            max_task = max(total_dict[valid_robots[0]], key=total_dict[valid_robots[0]].get)
            optimal_assignment = {valid_robots[0]: max_task}
            
        self.get_logger().info(str(optimal_assignment))

        # # 構建成本矩陣 (取負的成功率)
        # robots_with_tasks = [robot for robot in total_dict if total_dict[robot]]
        # tasks = list(set().union(*total_dict.values()))

        # cost_matrix = np.full((len(robots_with_tasks), len(tasks)), float('inf'))
        # for i, robot in enumerate(robots_with_tasks):
        #     for j, task in enumerate(tasks):
        #         if task in total_dict[robot]:
        #             cost_matrix[i, j] = -total_dict[robot][task]  # Use negative success rate as cost

        # # 使用匈牙利演算法求解最佳分配
        # row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # optimal_assignment = {robots_with_tasks[i]: tasks[j] for i, j in zip(row_ind, col_ind)}
        # self.get_logger().info(optimal_assignment)

        self.get_logger().info("Best task assignment :")
        for robot, task in optimal_assignment.items():
            self.get_logger().info(f"{robot} -> {task}")
        self.get_logger().info('\n------------------------------ [Finish] Task allocation ------------------------------')
        return optimal_assignment
    
    # 函數：顯示格式化的成功率表格
    def display_success_rate_table(self, total_dict, assignment):
        self.get_logger().info('\n---------- [Start] Display success rate table ----------')
        # 創建一個 DataFrame
        df = pd.DataFrame(total_dict).T

        # 四捨五入成功率到小數點後5位
        df = df.round(self.display_num)

        # 標記最佳配對
        for robot, task in assignment.items():
            df.at[robot, task] = '*' + str(df.at[robot, task])

        # 打印表格
        self.get_logger().info('\n{}'.format(df))
        self.get_logger().info('\n---------- [Finish] Display success rate table ----------')
    
    def get_target_task(self, task_string, target):
        self.get_logger().info(f'target task : {target}')
        tasks = task_string.split(';')
        for task in tasks:
            # 用 ':' 分隔任務名稱和任務資訊
            task_name, task_info = task.split(':')
            
            if task_name == target:
                self.get_logger().info(f'target string : {task}')
                return task
                       
    # 向參與實驗的發送新任務
    def send_new_task(self, task_list, optimal_assignment):
        self.get_logger().info('\n ------------------------------ [Start] Send new task ------------------------------')
        cli_list = [self.cli_for_rbA_nt, 
                    self.cli_for_rbB_nt, 
                    self.cli_for_rbC_nt, 
                    self.cli_for_rbD_nt, 
                    self.cli_for_rbE_nt] 
        
        
        for robot in self.attend_robot:
            if robot in self.robot_name_list:
                p = self.robot_name_list.index(robot)
                # 確認 srv 是否啟動
                while not cli_list[p].wait_for_service(timeout_sec=0.2):
                    self.get_logger().info(f'{robot} task recive agent service not available, waiting again...')
                
                # 根據當前要寄送的機器人選出對應的任務
                target_task = None
                for as_robot, as_task in optimal_assignment.items():
                    if as_robot == robot:
                        target_task = as_task
                
                task_string = self.get_target_task(task_list, target_task)
                
                self.req_nt.new_task_string = task_string
                future = cli_list[p].call_async(self.req_nt)
                self.get_logger().info(f'Send new task to {robot}_task_recive_agent. ')
                rclpy.spin_until_future_complete(self.sub_node, future)
                response = future.result()
                self.get_logger().info('Recive {}_task_recive_agent feedback : '.format(robot, response.feedback_msg))
                
        self.get_logger().info('\n ------------------------------ [Finish] Send new task ------------------------------')
    
        
    def node_action(self, request, response):
        recive_task_list = request.task_list
        self.get_logger().info('Recive client msg : {}'.format(recive_task_list))

        # 確認參與實驗的機器人有哪些
        if self.attend_robot == []:
            self.check_attend_robot()

        # 請求成功率 , 格式 : srst_list = [rb1_sr, rb2_sr, None, None, None]
        srst_list = self.requset_task_successrate(recive_task_list)
        # 將 srst_list 從 list 整理成 dict
        total_dict = self.srst_list2dict(srst_list)
        # 分配任務, 輸出格式(dict) : {'robot_A': 'task_5', 'robot_B': 'task_3' ....}
        optimal_assignment = self.task_assignment(total_dict)
        self.display_success_rate_table(total_dict, optimal_assignment)
        # 發送新的任務名稱與資訊給所有參與實驗的機器人
        self.send_new_task(request.task_list, optimal_assignment)

        rp_str = ''
        for rp_robot, rp_task in optimal_assignment.items():
            rp_str = (rp_str + f"{rp_robot}:{rp_task}")
        
        response.best_assignment = rp_str
        self.get_logger().info('New Task has been assignment , send the feedback to auctioneer agent.')
        
        return response 

def main(args=None):
    rclpy.init(args=args)
    agent = AllocationAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
