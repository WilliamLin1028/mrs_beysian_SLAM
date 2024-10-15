import rclpy
import os

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

from nav2_simple_commander.robot_navigator import BasicNavigator

from ament_index_python.packages import get_package_share_directory
# from mric_action.action import  GetSuccessrate

from pgmpy.models import BayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination     # Infering the posterior probability

import networkx as nx
import pylab as plt
import yaml
import numpy as np
import time
from scipy.spatial import distance

'''
此程式分成兩個部分,分別是server與publisher,其功能如下
server: 當程式收到action訊息時,計算 task array 中尚未完成的任務的successrate,並回傳紀錄task array successrate 的string
publisher: 當程式收到ongoing_task訊息時,計算 當前任務的的successrate, 並發佈到 topic - /task_ongoing_successrate 上
'''

class CalSuccessrateAgent(Node):
    def __init__(self):
        super().__init__('successrate_agent_node')
        self.sub_battery = self.create_subscription(Float32,'/battery', self.battery_status_callback,10)
        self.sub_task_ongoing = self.create_subscription(String,'/ongoing_task', self.ongoing_task_callback,10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub_task_ongoing_successrate = self.create_publisher(Float32, '/ongoing_task_successrate', 10)
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        # 載入"機器人設定"與"貝式網路 cpd" 的 yaml 路徑
        bringup_dir = get_package_share_directory('mric_slamtool')
        self.robot_config_yaml = os.path.join(bringup_dir, 'config', 'robot_config.yaml')
        self.BN_config_yaml = os.path.join(bringup_dir, 'config', 'my_cpdTable.yaml')
        self.task_array_txt = os.path.join(bringup_dir, 'database', 'task_list.txt')
        self.successrate_txt = os.path.join(bringup_dir, 'database', 'successrate.txt')
        self.cmd_request_sr_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_request_successrate.txt')
        # self.task_ongoing_txt = os.path.join(bringup_dir, 'database', 'ongoing.txt')
        
        self.BetteryThreshold = [12.0, 11.4] # 電池在實際運行時的電壓介於12.6V ~ 10.8V，在此每0.6V分成一個 level，共3階2個閥值
        # self.CostThreshold = [3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42] # Cost每3秒設定一個閥值，共15階14閥值
        self.CostThreshold = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20] # Cost每2秒設定一個閥值，共11階10閥值
        
        self.robot_name = None
        self.robot_type = None
        self.robot_skill = None
        
        self.battery = None
        self.ongoing_task = None
        
        self.robot_speed = 0.10     # 0.2 meter per second
        self.buffer = 1.2           # 我們假設到達目標的實際時間是預測時間的 1.2 倍
        
        self.show_robot_status = False                # 不顯示機器人狀態
        self.show_successrate_cal_process = False      # 不顯示成功率計算過程
        
        self.position_x = None
        self.position_y = None
        self.orientation = None
        
        self.last_dict = None      # 0916 addin
        
        self.nav = BasicNavigator()
        # self.nav.waitUntilNav2Active()
        self.model = None
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.node_action)
                 
            
    def publish_termial_msg(self, msg):
        if self.show_successrate_cal_process == True:
            self.get_logger().info(msg)
    
    # build_BayesianNetwork_structure 和 load_robotconfig 只會在最開始啟動時執行，並用來載入"機器人基礎能力"設定與"BayesianNetwork"設定
    def build_BayesianNetwork_structure(self):

        '''
        This code's BayesianNetwork structure
                            NodeE ─────┐ 
        NodeA ───┐      ┌── NodeD ─── NodeF ─────┐
                NodeC ──┼───────────────────── NodeH
        NodeB ───┘      └── NodeG ───────────────┘
            NodeJ ── NodeK ───┘

        '''
        
        if self.model == None:
            self.publish_termial_msg('[build_BayesianNetwork_structure] ---- start.')
            self.publish_termial_msg('Start loading BayesianNetwork config......')
            #self.get_logger().info('[build_BayesianNetwork_structure] ---- start.')
            #self.get_logger().info('Start loading BayesianNetwork config......')
            # Defining Bayesian Structure
            self.model = BayesianNetwork([('NumTask', 'Task'), ('Robot', 'Task'),
                                        ('Task', 'Cost'), ('Task', 'Success'), ('Task', 'AbilitySatisfaction'),
                                        ('Cost', 'CostSatisfaction'), ('Battery', 'CostSatisfaction'), ('CostSatisfaction', 'Success'),
                                        ('Skill', 'SkillCond'), ('SkillCond', 'AbilitySatisfaction'), ('AbilitySatisfaction', 'Success')])

            # Defining the CPDs:
            # NodeA - NumTask
            cpd_NumTask = self.load_BayesianNetwork('cpd_NumTask')

            # NodeB - Robot
            cpd_Robot = self.load_BayesianNetwork('cpd_Robot')

            # NodeC - Task
            # TabularCPD('node_name', node_class_num , [[class_0 array], ... ,[class_num array]], evidence=[source mode], evidence_card=[source node class num])
            cpd_Task = self.load_BayesianNetwork('cpd_Task')

            # NodeD  - Cost **Cost 會隨時間改變, 在動態環境下,如果發現當前successrate下降到一定程度時需要觸發重新選擇任務
            cpd_Cost = self.load_BayesianNetwork('cpd_Cost')

            # NodeE - Battery
            cpd_Battery = self.load_BayesianNetwork('cpd_Battery')

            # NodeF - CostSatisfaction
            cpd_CostSatisfaction = self.load_BayesianNetwork('cpd_CostSatisfaction')

            # NodeJ - Skill(drive)
            cpd_Skill = self.load_BayesianNetwork('cpd_Skill')  # 機器人是否有駕駛移動能力的機率

            # NodeK - SkillCond
            cpd_SkillCond = self.load_BayesianNetwork('cpd_SkillCond')

            # NodeG - AbilitySatisfaction
            cpd_AbilitySatisfaction = self.load_BayesianNetwork('cpd_AbilitySatisfaction')

            # NodeH - Success
            cpd_Success = self.load_BayesianNetwork('cpd_Success')

            #print(cpd_Success)

            # Associating the CPDs with the network structure.
            self.model.add_cpds(cpd_NumTask, cpd_Robot, cpd_Task, cpd_Cost, cpd_Battery, cpd_CostSatisfaction, cpd_Skill, cpd_SkillCond, cpd_AbilitySatisfaction, cpd_Success)

            self.model.check_model()
            
            # self.get_logger().info('[build_BayesianNetwork_structure] ---- completion.')
            self.publish_termial_msg('[build_BayesianNetwork_structure] ---- completion.')

    def load_BayesianNetwork(self, cpdName):
        self.publish_termial_msg('[load_BayesianNetwork] ---- start.')
        self.publish_termial_msg('Start loading {} ...'.format(cpdName))
        #self.get_logger().info('[load_BayesianNetwork] ---- start.')
        #self.get_logger().info('Start loading {} ...'.format(cpdName))
        
        NodeName = None
        ClassNum = None
        CPDarray = []
        Evidence = None
        Evidence_card = None
        # BayesianNetwork 的檔案路徑
        # yaml_path = r'/content/basic_cpdTable.yaml'

        f = open(self.BN_config_yaml, 'r', encoding='utf-8')
        cpdTable = yaml.load(f.read(), Loader=yaml.FullLoader)
        f.close()

        for key, value in cpdTable.get(cpdName).items():
            if key == 'NodeName':
                NodeName = value
            elif key == 'ClassNum':
                ClassNum = value
            elif key == 'CPDarray':
                for arrayKey, arrayValue in value.items():
                    CPDarray.append(arrayValue)
            elif key == 'Evidence':
                Evidence = value
            elif key == 'Evidence_card':
                Evidence_card = value
                
        CPD = TabularCPD(NodeName, ClassNum, CPDarray, evidence = Evidence, evidence_card = Evidence_card)
        
        self.publish_termial_msg(' {} loading completed .'.format(cpdName))
        self.publish_termial_msg('[load_BayesianNetwork] ---- completion.')
        
        return CPD

    # 用一個action取得機器人當前的相關資訊，包含"機器人類型"、
    def load_robotconfig(self):
        if any(var is None for var in [self.robot_name, self.robot_type, self.robot_skill]):
            self.publish_termial_msg('[load_robotconfig] ---- start.')
            self.publish_termial_msg('Start loading  Robot config ...')
            
            f = open(self.robot_config_yaml, 'r', encoding='utf-8')
            robot_config = yaml.load(f.read(), Loader=yaml.FullLoader)
                # self.get_logger().info('{}'.format(f))
            f.close()
            
            for key, value in robot_config.items():
                if key == 'Robot_name':
                    self.publish_termial_msg('Robot_name : {}'.format(value))
                    self.robot_name = value
                elif key == 'Robot_type':
                    self.publish_termial_msg('Robot_type : {}'.format(value))
                    if value == 'ST':
                        self.robot_type = 0
                    else :
                        self.robot_type = 1
                elif key == 'Robot_skill':
                    self.publish_termial_msg('Robot_skill : {}'.format(value))
                    if value == 'Yes':
                        self.robot_skill = 0
                    else :
                        self.robot_skill = 1
                    
            self.publish_termial_msg('Robot_name : {}'.format(self.robot_name))
            self.publish_termial_msg('Robot_type : {}'.format(self.robot_type))
            self.publish_termial_msg('Robot_skill : {}'.format(self.robot_skill))
            self.publish_termial_msg('[load_robotconfig] ---- completion.')

    def battery_status_callback(self, msg):
        self.battery = float(msg.data)
        
    def ongoing_task_callback(self, msg):
        self.ongoing_task = msg.data
    
    def odom_callback(self, msg):
        # self.publish_termial_msg("Received odometry message:\n%s" % msg)
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.orientation = msg.pose.pose.orientation
    
    def onging_task_successrate(self):
        if self.ongoing_task != None:
        # 初始化空的 dictionary
            tasks_dict = {}
            self.publish_termial_msg('[ongoing_task] {}'.format(self.ongoing_task))
            # 用 ';' 分隔不同的任務
            # tasks = ongoing_task.split(';')
            
            if ':' in self.ongoing_task and ',' in self.ongoing_task:
                # 用 ':' 分隔任務名稱和任務資訊
                task_name, task_info = self.ongoing_task.split(':')
                
                # 用 ',' 分隔任務資訊的細項
                task_status, x_pose, y_pose = task_info.split(',')
                
                # 將資訊存入 dictionary
                tasks_dict[task_name] = {
                    'status': task_status,
                    'x_pose': float(x_pose),
                    'y_pose': float(y_pose),
                    'cost': None
                }
            
            tasks_dict = self.get_cost(tasks_dict)                  # 經過該函數只會獲得cost資訊,單位:秒
            tasks_dict_BN = self.cost_trans2_level(tasks_dict)      # 經過該函數會將BayesianNetwork所需的所有input資訊,轉換成對應的格式
            successrate_list = self.calculate_successrate(tasks_dict_BN)    # 經過該函數successrate_list只會剩任務名稱以及其任務成功率
            self.publish_termial_msg('[pub_successrate] ---- start.')
            msg = Float32()
            self.publish_termial_msg('{}:{}'.format(successrate_list[0][0],successrate_list[0][1]))
            msg.data = successrate_list[0][1]
            # pose_msg = self.successrate_dict2string(successrate_dict)       # 經過該函數會將successrate_dict 轉換成用來發布或儲存的字串
            self.pub_task_ongoing_successrate.publish(msg)
            self.publish_termial_msg('[pub_successrate] ---- completion.')
        
    def load_taskarray(self):
        self.publish_termial_msg('[load_taskarray] ---- start.')
        self.publish_termial_msg('Start loading task array file.')
        # 確認檔案是否已被開啟的假檔案
        lock_file = self.task_array_txt + '.lock'

        # 等待直到檔案被釋放
        if os.path.exists(lock_file):
            self.publish_termial_msg('Task array file is currently in use, waiting...')
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

            # 至字串格式 {task_name}:{status},{x_pose},{y_pose};{task_name}:{status},{x_pose},{y_pose};...
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
        
        self.publish_termial_msg('[load_taskarray] ---- completion.')
        
        return tasks_dict

    def cost_trans2_level(self, tasks_dict):
        self.publish_termial_msg('[trans2_level] ---- start.')
        self.publish_termial_msg('Transfor Cost, Battery, Numtask .ect information to level for BayesianNetwork.')
        self.publish_termial_msg('Start transform information...')
        battery_level = 0
        cost_level = 0
        numtask_level = 0
        new_tasks_dict = {}
        
        # 取得當下機器人的電池 level
        for i in range(len(self.BetteryThreshold)):
            if self.battery >= self.BetteryThreshold[i]:
                battery_level += 1
                        
        # 當機器人還沒完成當前任務時,numtask_level (機器人所持有的任務數量) 為 1 
        while not self.nav.isTaskComplete():
            numtask_level = 1
        
        # 將後續用於計算 BayesianNetwork 的資訊打包進 new_tasks_dict 中
        for task_name, task_details in tasks_dict.items():
            if tasks_dict[task_name]['status'] == 'unexplored':
                cost = tasks_dict[task_name]['cost']
                cost_level = 0
                for i in range(len(self.CostThreshold)):
                    if cost >= self.CostThreshold[i]:
                        cost_level += 1
                        
                new_tasks_dict[task_name] = {
                'NumTask': numtask_level,           # NumTask 只有 0 和 1 兩個值,日後進行MTMR實驗後可自行挑調整
                'Robot': self.robot_type, 
                'Cost':cost_level, 
                'Battery': battery_level, 
                'Skill': self.robot_skill
                }
                
        # self.publish_termial_msg('battery_level: {} ,cost_level: {}\n'.format(battery_level, cost_level))
        self.publish_termial_msg('Information transforming completed.')
        self.publish_termial_msg('[trans2_level] ---- completion.')
        return new_tasks_dict

    def successrate_list2string(self, successrate_list):
        # 此過程產生的字串格式如下: task_1:0.67875...;task_2:0.54573...;task_3:0.87664...
        self.publish_termial_msg('[dict2string] ---- start.')
        msg_string = ''
        for i in range(len(successrate_list)):
            self.publish_termial_msg('{}:{}'.format(successrate_list[i][0],successrate_list[i][1]))
            if msg_string != '':
                msg_string = msg_string+';{}:{}'.format(successrate_list[i][0],successrate_list[i][1])
            elif msg_string == '':
                msg_string = msg_string+'{}:{}'.format(successrate_list[i][0],successrate_list[i][1])
        
        # for task_name, task_details in tasks_dict.items():
        #     self.publish_termial_msg('{}:{}'.format(tasks_dict[task_name],task_details['successrate']))
        #     if msg_string != '':
        #         msg_string = msg_string+';{}:{}'.format(tasks_dict[task_name],
        #                                             task_details['successrate'])
        #     elif msg_string == '':
        #         msg_string = msg_string+'{}:{}'.format(tasks_dict[task_name],
        #                                             task_details['successrate'])
        
        self.publish_termial_msg('[dict2string] ---- completion.')
        return msg_string

    # 這段函數是用來取得 Cost 的,在本實驗中所設定的 cost 為到達目標時間
    def get_cost(self, tasks_dict):
        self.publish_termial_msg('[get_cost] ---- start.')
        

        init_x_pose = float(self.position_x)
        init_y_pose = float(self.position_y)
        self.publish_termial_msg(f'Robot is located at ({init_x_pose},{init_y_pose})')
        
        for task_name, task_details in tasks_dict.items():
            if tasks_dict[task_name]['status'] == 'unexplored':
                goal_x_pose = tasks_dict[task_name]['x_pose']
                goal_y_pose = tasks_dict[task_name]['y_pose']
                
                self.publish_termial_msg('start pose : {}, {} ; goal pose : {}, {}'.format(init_x_pose, init_y_pose, goal_x_pose, goal_y_pose))
                initial_pose = PoseStamped()
                initial_pose.header.frame_id = 'map'
                initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
                initial_pose.pose.position.x = init_x_pose
                initial_pose.pose.position.y = init_y_pose
                initial_pose.pose.orientation.z = 0.707
                initial_pose.pose.orientation.w = 0.707
                
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
                goal_pose.pose.position.x = goal_x_pose
                goal_pose.pose.position.y = goal_y_pose
                goal_pose.pose.orientation.w = 0.707
                goal_pose.pose.orientation.z = 0.707
                # goal_pose.append(goal_pose1)
                    
                path = self.nav.getPath(initial_pose, goal_pose)  # 取得到達目標點的路徑
                while path is None or path.poses is None:
                    time.sleep(0.1)
                    path = self.nav.getPath(initial_pose, goal_pose)  # 取得到達目標點的路徑
                poses_list = path.poses     # 路徑是以list存入
                pose_1 = None
                pose_2 = None
                distance_list =[]
                for i in poses_list:
                    if pose_1 == None:
                        pose_1 = i.pose.position
                    else:
                        pose_2 = i.pose.position
                        a = (pose_1.x, pose_1.y)
                        b = (pose_2.x, pose_2.y)  
                        distance_list.append(distance.euclidean(a, b))
                        pose_1 = i.pose.position
                
                total_distance = sum(distance_list)
                self.publish_termial_msg('total distance : {}'.format(total_distance))
                
                cost = (total_distance / self.robot_speed) * self.buffer
                
                tasks_dict[task_name]['cost'] = cost
            
            self.publish_termial_msg('[get_cost] ---- completion.')
            
        return tasks_dict

    def calculate_successrate(self, tasks_dict):
        self.publish_termial_msg('[calculate_successrate] ---- start.')
        self.publish_termial_msg('Start calculate successrate.')
        
        successrate_list = []
        infer = VariableElimination(self.model)
        
        # 再計算successrate 前,先輸出 dict
        for task_name, task_details in tasks_dict.items():
            self.publish_termial_msg('{}:'.format(task_name))
            self.publish_termial_msg('\t NumTask: {}'.format(task_details['NumTask']))
            self.publish_termial_msg('\t Robot: {}'.format(task_details['Robot']))
            self.publish_termial_msg('\t Cost: {}'.format(task_details['Cost']))
            self.publish_termial_msg('\t Battery: {}'.format(task_details['Battery']))
            self.publish_termial_msg('\t Skill: {}'.format(task_details['Skill']))
            
        
        for task_name, task_details in tasks_dict.items():
            posterior_p1 = infer.query(['Task', 'Success'], evidence={'NumTask': task_details['NumTask'], 
                                                                      'Robot': task_details['Robot'], 
                                                                      'Cost':task_details['Cost'], 
                                                                      'Battery': task_details['Battery'], 
                                                                      'Skill': task_details['Skill']})
            successrate_table = posterior_p1.values
            successrate = successrate_table[0, 0]
            successrate_list.append([f'{task_name}',successrate])

            self.publish_termial_msg(f'{task_name} : {successrate}')
        
        self.publish_termial_msg(f'{successrate_list}')
        self.publish_termial_msg('Successrate completion calculation.')
        self.publish_termial_msg('[calculate_successrate] ---- completion.')
        
        return successrate_list
        
    def save_successrate_list(self, file_path, successrate_string):
        self.publish_termial_msg('[save_successrate] ---- start.')
        with open(file_path, 'w') as f:
            f.write(successrate_string)
        self.publish_termial_msg('[save_successrate] ---- completion.')
    
    def pub_task_mark_array(self, tasks_dict):
        marker_array = MarkerArray()
        # tasks_dict = self.load_taskarray()                              # 載入 task list 並將 string 轉換成 dict
        i = 0

        for task_name, task_details in tasks_dict.items():
            if tasks_dict[task_name]['status'] == 'unexplored':
                goal_x_pose = tasks_dict[task_name]['x_pose']
                goal_y_pose = tasks_dict[task_name]['y_pose']
                
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "points"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = goal_x_pose
                marker.pose.position.y = goal_y_pose
                marker.pose.position.z = 0.0
                
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.2
                
                marker.color.a = 1.0  # 透明度
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                
                marker_array.markers.append(marker)
                
                # 創建文字標記
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = "text"
                text_marker.id = i + 200  # 確保ID不同於點標記
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                # 文字標記的位置略微提高
                text_marker.pose.position.x = marker.pose.position.x
                text_marker.pose.position.y = marker.pose.position.y
                text_marker.pose.position.z = marker.pose.position.z + 0.3  # 提高文字標記的位置
                
                text_marker.pose.orientation.x = 0.0
                text_marker.pose.orientation.y = 0.0
                text_marker.pose.orientation.z = 0.0
                text_marker.pose.orientation.w = 1.0
                
                text_marker.scale.z = 0.1  # 設置文字大小
                
                text_marker.color.a = 1.0  # 透明度
                text_marker.color.r = 0.0
                text_marker.color.g = 1.0
                text_marker.color.b = 0.0
                
                text_marker.text = "Point_{}".format(i+1)  # 設置文字內容
                
                marker_array.markers.append(text_marker)
                i +=1 

        
        self.publisher.publish(marker_array)

    def node_action(self):
        # 預計將本程式分成 2 部分,分別是 ongoing_task 與 total_task
        if self.position_x == None or self.position_y == None:
            self.publish_termial_msg('Wait for position info ...')
            return
        
        if os.path.exists(self.cmd_request_sr_file_path) and not os.path.exists(self.successrate_txt):
            # 若 cmd_request_sr_file_path 存在, 但成功率不存在時,進行 task list 成功率計算
            self.publish_termial_msg('Task list exist, but successrate file is not exist. Start calculated total success rate.')
            tasks_dict = self.load_taskarray()                              # 載入 task list 並將 string 轉換成 dict
            self.pub_task_mark_array(tasks_dict)
            tasks_dict = self.get_cost(tasks_dict)                          # 經過該函數只會獲得cost資訊,單位:秒
            tasks_dict_BN = self.cost_trans2_level(tasks_dict)              # 經過該函數會將BayesianNetwork所需的所有input資訊,轉換成對應的格式
            successrate_list = self.calculate_successrate(tasks_dict_BN)    # 經過該函數successrate_dict只會剩任務名稱以及其任務成功率
            pose_msg = self.successrate_list2string(successrate_list)       # 經過該函數會將successrate_dict 轉換成用來發布或儲存的字串
            self.save_successrate_list(self.successrate_txt, pose_msg)      # 將成功率儲存到檔案中
        else: 
            # 否則根據 sub_task_ongoing 的資訊計算任務成功率
            if self.ongoing_task == None:
                return
            self.onging_task_successrate()


def main(args=None):
    rclpy.init(args=args)
    agent = CalSuccessrateAgent()
    agent.publish_termial_msg('inside_successrate_agent start working ...')
    agent.get_logger().info('Trigger navigator ...')
    agent.__init__()
    agent.get_logger().info('Loading Robot config ...')
    agent.load_robotconfig()
    agent.get_logger().info('Building BayesianNetwork structure ...')
    agent.build_BayesianNetwork_structure()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()