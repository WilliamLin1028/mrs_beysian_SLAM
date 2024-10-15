import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32
from ament_index_python.packages import get_package_share_directory

import os
import time
import random
import numpy as np

'''
此檔案是在 server 內用來控制當前機器人的移動狀態的，分成兩部分
ongoing task 資料格式如下:
{task_name}:{x_pose},{y_pose}

A. 當機器人沒有任務部目標時
1. 建立 robot_standby.txt 檔案,並等待 ongoing_task.txt 出現
2. 透過讀取 ongoing_task.txt 向 GoPose設定 移動目標

B. 當機器人正在移動到目標時
1. 建立 robot_ongoing.txt 檔案
2. 當機器人到達目標或是取消任務時,移除robot_ongoing.txt 檔案
'''


class Executor(Node):
    def __init__(self):
        super().__init__('executor_node')
        bringup_dir = get_package_share_directory('mric_slamtool')
        self.robot_standby_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_robot_standby.txt')
        self.robot_ongoing_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_robot_ongoing.txt')
        self.update_task_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_update_ongoing_task.txt')
        
        self.task_file_path = os.path.join(bringup_dir, 'database', 'ongoing_task.txt')
        self.fail_task_file_path = os.path.join(bringup_dir, 'database', 'fail_task.txt')
        self.complete_task_file_path = os.path.join(bringup_dir, 'database', 'complete_task.txt')
        
        self.navigator = BasicNavigator()
        
        self.task_str = None
        
        self.successrate_threshold = 0.1 # 用來防止機器人執迷不悟的執行成功率越來越低的任務
        self.cancel_task_by_sr = False
        self.count = 0
        self.count_b = 0
        self.uknow_result_count = 0
        self.anti_uknow_result = 80      # 與 uknow_result_count 配合,如果uknow_result_count累計的次數出大於anti_uknow_result,即取消任務
        self.printout_count = 30000

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.node_action)
        self.pub_ongoing_task = self.create_publisher(String, '/ongoing_task', 10)
        self.sub_ongoing_task_successrate = self.create_subscription(Float32,
                                                                     '/ongoing_task_successrate', 
                                                                     self.breakdown_manager,
                                                                     10)
        
        
    def check_file_existence(self, file_path):
        """
        參數:   directory (str): 要檢查的資料夾路徑
                filename (str): 要檢查的檔案名稱
        回傳:   bool: 檔案是否存在的訊息
        """
        if os.path.isfile(file_path):
            # self.get_logger().info(f'{file_path} file is exist')
            return True
        else:
            # self.get_logger().info(f'{file_path} file is not exist')
            return False
    
    
    def read_task_info(self, file_path):
        # 確認檔案是否已被開啟的假檔案
        lock_file = file_path + '.lock'
        self.get_logger().info('Read ongoing_task.txt file')
        if not os.path.exists(lock_file):
            open(lock_file, 'w').close()
            
            task_file_exist_check = self.check_file_existence(file_path)
            while task_file_exist_check == False:
                task_file_exist_check = self.check_file_existence(file_path)
                
            with open(file_path, 'r') as file:
                task_string = file.read().strip()
                
            os.remove(lock_file)
            """
            從給定的字串中提取 x_pose 和 y_pose
            字串格式: {task_name}:{status},{x_pose},{y_pose}
            """
            # 用於 publish 當前任務的資訊給 inside_successrate_agent.py 計算當前的任務成功率
            self.get_logger().info(f'task_string : {task_string}')
            self.task_str = task_string
            
            # 初始化 x_pose 和 y_pose 的變數
            # x_pose = None
            # y_pose = None
            # status = None

            # 分割 task_string 以提取座標
            if ':' in task_string and ',' in task_string:
                # 使用 ':' 分隔 task_name 和座標
                task_name, coordinates = task_string.split(':')
                # 使用 ',' 分隔 x_pose 和 y_pose
                status, x_pose, y_pose = coordinates.split(',')
                # 將 x_pose 和 y_pose 轉換為浮點數
                x_pose = float(x_pose)
                y_pose = float(y_pose)

            self.get_logger().info(f'task name : "{task_name}" ; status : "{status}" ; x_pose : "{x_pose}" ; y_pose : "{y_pose}"')
            
            return task_name, status, x_pose, y_pose
    
    
    def save_fail_task_file(self, file_path, fail_task_str):
        # 格式化配對結果為字串
        # fail_task_str = f'{task_name}:{status},{x_pose},{y_pose}'
        
        # 檢查檔案是否存在，如果存在則刪除
        if os.path.exists(file_path):
            self.get_logger().info('File is exist. Remove old version.')
            os.remove(file_path)
        
        # 將配對結果寫入檔案
        with open(file_path, 'w') as file:
            file.write(fail_task_str)
            self.get_logger().info('Created fail_task.txt.')
            
    def save_success_task_file(self, file_path, success_task_str):
        # 格式化配對結果為字串
        # fail_task_str = f'{task_name}:{status},{x_pose},{y_pose}'
        
        # 檢查檔案是否存在，如果存在則刪除
        if os.path.exists(file_path):
            self.get_logger().info('File is exist. Remove old version.')
            os.remove(file_path)
        
        # 將配對結果寫入檔案
        with open(file_path, 'w') as file:
            file.write(success_task_str)
            self.get_logger().info('Created complete_task.txt.')
    
    
    def send_goal_pose(self, task_x_pose, task_y_pose):
        if not self.navigator:
            return
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        # goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = task_x_pose
        goal_pose.pose.position.y = task_y_pose
        goal_pose.pose.orientation.w = random.random()

        self.get_logger().info('Goal pose: x={}, y={}'.format(task_x_pose, task_y_pose))
        self.navigator.goToPose(goal_pose)
        
        if os.path.isfile(self.robot_standby_file_path):
            os.remove(self.robot_standby_file_path)
        if not os.path.isfile(self.robot_ongoing_file_path):
            open(self.robot_ongoing_file_path, 'w').close() 
    
    
    # 用來向 inside_successrate_agent.py 請求當下任務的成功率
    def publish_msg(self, msg):
        # msg 格式: String, {task_name}:{status},{x_pose},{y_pose}
        self.pub_ongoing_task.publish(msg)
    
    def breakdown_manager(self, msg):
        task_successrate = msg.data # 數值介於 0 ~ 1.0 ,是浮點數
        if task_successrate < self.successrate_threshold:
            self.cancel_task_by_sr = True
    
    def initial_status(self):
        self.get_logger().info('Initial Robot Status ....')
        self.count = 0
        if self.check_file_existence(self.robot_ongoing_file_path):
            os.remove(self.robot_ongoing_file_path)
        if self.check_file_existence(self.robot_standby_file_path):
            os.remove(self.robot_standby_file_path)
        if self.check_file_existence(self.update_task_file_path):
            os.remove(self.update_task_file_path)
        #if self.check_file_existence(self.task_file_path):
        #    os.remove(self.task_file_path)
        if self.check_file_existence(self.task_file_path):
            os.remove(self.task_file_path)

    
    def node_action(self):
        # self.get_logger().info('navigator check : {}'.format(self.navigator.isTaskComplete()))
        
        # 當機器人完成任務或閒置時
        while self.navigator.isTaskComplete():
            if self.count_b == self.printout_count :
                self.count_b = 0
                self.get_logger().info('[Waiting Phase] Task complete check : {}'.format(self.navigator.isTaskComplete()))

            standby_message = self.check_file_existence(self.robot_standby_file_path)
            onging_message = self.check_file_existence(self.robot_ongoing_file_path)
            task_message = self.check_file_existence(self.task_file_path)
            #self.get_logger().info(f'standby file : {standby_message} ; onging file : {onging_message} ; task file : {task_message}')
            
            if os.path.isfile(self.robot_ongoing_file_path) ==True:
                os.remove(self.robot_ongoing_file_path)
                open(self.robot_standby_file_path, 'w').close() 
            
            if standby_message == False and onging_message == False:
                self.get_logger().info('cmd_robot_standby.txt is Not Exist')
                self.get_logger().info('cmd_robot_ongoing.txt is Not Exist')
                self.get_logger().info('Program is in initial stage.')
                self.get_logger().info('Created cmd_robot_standby.txt to trigger auctioneer.')
                open(self.robot_standby_file_path, 'w').close() 
                
            elif (standby_message == True) and (onging_message == False) and (task_message == False):
                if self.count == 0:
                    self.get_logger().info('cmd_robot_standby.txt is Not Exist')
                    self.get_logger().info('cmd_robot_ongoing.txt is Exist')
                    self.get_logger().info('ongoing_task.txt is Not Exist')
                    self.get_logger().info('Waiting for task ... ')
                    self.count += 1
                    
            elif (standby_message == True) and (onging_message == False) and (task_message == True):
                self.get_logger().info('Recive the task.')
                self.count = 0
                task_name, status, x_pose, y_pose = self.read_task_info(self.task_file_path)
                self.get_logger().info('Send the goal_pose to Nav2.')
                self.send_goal_pose(x_pose, y_pose)
                
                while not self.navigator.isTaskComplete():
                    if self.count == 0:
                        self.get_logger().info('Wait Nav2 Start Task......')
                        self.count += 1
                
                self.count = 0
                time.sleep(0.1)
                if self.check_file_existence(self.update_task_file_path):
                    self.get_logger().info('Remove cmd_update_ongoing_task.txt')
                    os.remove(self.update_task_file_path)
                self.get_logger().info('Start executing New Task ...')
               
            elif (standby_message == False) and (onging_message == True) and (task_message == True):
                if self.count == 0:
                    self.get_logger().info('cmd_robot_standby.txt is Not Exist')
                    self.get_logger().info('cmd_robot_ongoing.txt is Exist')
                    self.get_logger().info('ongoing_task.txt is Exist')
                    self.get_logger().info('Robot is on woking. ')
                    self.count += 1
                
            else:
                if self.count == 0:
                    self.get_logger().info('Status Error ......')
                    self.get_logger().info('cmd file check ......')
                    self.get_logger().info('cmd_robot_standby.txt : {}'.format(standby_message))
                    self.get_logger().info('cmd_robot_ongoing.txt : {}'.format(onging_message))
                    self.get_logger().info('ongoing_task.txt : {}'.format(task_message))
                    #self.get_logger().info('[Waiting Phase] Task complete check : {}'.format(self.navigator.isTaskComplete()))
                    self.count += 1
            
            self.count_b += 1
                
                
        # 當機器人正在執行任務時
        while not self.navigator.isTaskComplete() :
            if self.count_b == self.printout_count :
                self.count_b = 0
                self.get_logger().info('[Working Phase] Task complete check : {}'.format(self.navigator.isTaskComplete()))
                self.get_logger().info('Test')
            # 如果在運行中被分配到新任務，則取消舊任務執行新任務
            if os.path.isfile(self.update_task_file_path) and os.path.isfile(self.task_file_path):
                self.get_logger().info('Recive the new task. Cancel current task and execute new task')
                task_name, status, x_pose, y_pose = self.read_task_info(self.task_file_path)
                self.navigator.cancelTask()
                self.send_goal_pose(x_pose, y_pose)
                # 在成功新任務後刪除 command 文件
                os.remove(self.update_task_file_path)
            
            # 發布正在執行的任務
            pub_msg = String()
            pub_msg.data = self.task_str
            self.publish_msg(pub_msg)
            # 如果接收到的回傳的任務成功率低於門檻值，則取消該任務重新進行任務分配
            if self.cancel_task_by_sr == True : 
                self.get_logger().info('Success rate below threshold .Goal was Canceled!')
                if self.check_file_existence(self.robot_ongoing_file_path):
                    os.remove(self.robot_ongoing_file_path)
                open(self.robot_standby_file_path, 'w').close()
                self.cancel_task_by_sr = False
                return
            
            # 如果再執行中沒有出現新任務   
            result = self.navigator.getResult()
            self.get_logger().info('{}'.format(result))
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'\n========================================= [TaskResult] =========================================')
                self.get_logger().info('Goal Succeeded!')
                task_name, status, x_pose, y_pose = self.read_task_info(self.task_file_path)
                complete_task_str = f'{task_name}:{status},{x_pose},{y_pose}'
                self.get_logger().info(f'Completed {complete_task_str}')
                self.get_logger().info(f'\n-------------------------------------- [Save Success Task] --------------------------------------')
                self.save_success_task_file(self.complete_task_file_path, complete_task_str)
                self.get_logger().info(f'\n----------------------------------- [Remove Task and Cmd File] -----------------------------------')     
                if self.check_file_existence(self.robot_ongoing_file_path):
                    os.remove(self.robot_ongoing_file_path)
                if self.check_file_existence(self.task_file_path):
                    os.remove(self.task_file_path)
                open(self.robot_standby_file_path, 'w').close()
                self.uknow_result_count = 0  
                
            elif result == TaskResult.CANCELED:
                self.get_logger().info(f'\n========================================= [TaskResult] =========================================')
                self.get_logger().info('Goal was Canceled!')
                task_name, status, x_pose, y_pose = self.read_task_info(self.task_file_path)
                fail_task_str = f'{task_name}:{status},{x_pose},{y_pose}'
                self.get_logger().info(f'Cancel {fail_task_str}')
                self.save_fail_task_file(self.fail_task_file_path, fail_task_str)
                if self.check_file_existence(self.robot_ongoing_file_path):
                    os.remove(self.robot_ongoing_file_path)
                if self.check_file_existence(self.task_file_path):
                    os.remove(self.task_file_path)
                open(self.robot_standby_file_path, 'w').close()
                self.navigator.cancelTask()
                self.uknow_result_count = 0
                
                
            elif result == TaskResult.FAILED:
                self.get_logger().info(f'\n========================================= [TaskResult] =========================================')
                self.get_logger().info('Goal Failed!')
                task_name, status, x_pose, y_pose = self.read_task_info(self.task_file_path)
                fail_task_str = f'{task_name}:{status},{x_pose},{y_pose}'
                self.get_logger().info(f'Cancel {fail_task_str}')
                self.save_fail_task_file(self.fail_task_file_path, fail_task_str)
                if self.check_file_existence(self.robot_ongoing_file_path):
                    os.remove(self.robot_ongoing_file_path)
                if self.check_file_existence(self.task_file_path):
                    os.remove(self.task_file_path)
                open(self.robot_standby_file_path, 'w').close()
                self.navigator.cancelTask()
                self.uknow_result_count = 0
                
            elif result == TaskResult.UNKNOWN:
                self.get_logger().info('Goal Result Unknown!')
                self.uknow_result_count +=1
                if  self.uknow_result_count == self.anti_uknow_result: 
                    self.get_logger().info(f'\n========================================= [TaskResult] =========================================')
                    self.get_logger().info('Too Many Result Unknown!')
                    task_name, status, x_pose, y_pose = self.read_task_info(self.task_file_path)
                    fail_task_str = f'{task_name}:{status},{x_pose},{y_pose}'
                    self.get_logger().info(f'Cancel {fail_task_str}')
                    self.save_fail_task_file(self.fail_task_file_path, fail_task_str)
                    if self.check_file_existence(self.robot_ongoing_file_path):
                        os.remove(self.robot_ongoing_file_path)
                    if self.check_file_existence(self.task_file_path):
                        os.remove(self.task_file_path)
                    open(self.robot_standby_file_path, 'w').close()
                    self.navigator.cancelTask()
                    self.uknow_result_count = 0
                
            else:
                self.get_logger().info('Goal has an invalid return status!')
                #task_name, status, x_pose, y_pose = self.read_task_info(self.task_file_path)
                #fail_task_str = f'{task_name}:{status},{x_pose},{y_pose}'
                #self.get_logger().info(f'Cancel {fail_task_str}')
                #self.save_fail_task_file(self.fail_task_file_path, fail_task_str)
                #if self.check_file_existence(self.robot_ongoing_file_path):
                #    os.remove(self.robot_ongoing_file_path)
                #if self.check_file_existence(self.task_file_path):
                #    os.remove(self.task_file_path)
                #open(self.robot_standby_file_path, 'w').close()
                #self.navigator.cancelTask()

            self.count_b += 1



def main(args=None):
    rclpy.init(args=args)
    agent = Executor()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
