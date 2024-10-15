import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

from ament_index_python.packages import get_package_share_directory

from mric_slamtool_interfaces.srv import ConnectReciver, UpdateTaskList, AllocateTask

import os
import time
import random
import yaml
import numpy as np


robot_name = 'robot_D'

class TaskAgent(Node):
    def __init__(self):
        super().__init__(f'{robot_name}_task_agent_node')
        
        self.srv = self.create_service(UpdateTaskList, f'{robot_name}_update_task', self.node_action)
        
        bringup_dir = get_package_share_directory('mric_slamtool')
        # 確認參與機器人指令檔案的資料夾
        self.attend_cmd_file_path = os.path.join(bringup_dir, 'cmd_folder')
        self.database_path = os.path.join(bringup_dir, 'database')
        
        self.task_list_file_path = os.path.join(bringup_dir, 'database', 'task_list.txt')
        self.ongoing_task_file_path = os.path.join(bringup_dir, 'database', 'ongoing_task.txt')
        self.goal_fail_file = os.path.join(bringup_dir, 'database', 'fail_task.txt')
        
        # 溝通設定
        self.subscription_map = self.create_subscription(OccupancyGrid, f'/{robot_name}_merge_map', self.map_callback, 10)
        self.merge_map = None
        self.map_explored = None
  
        self.explore_space = [2.55, 2.7, -0.15, -0.9]     # 探索範圍 [x_max, y_max, x_min, y_min] 
        self.task_num = 13 
    

    def read_ongoing_fail_task(self, file_path):
        # 確認檔案是否已被開啟的假檔案
        lock_file = file_path + '.lock'
        if not os.path.exists(lock_file):
            open(lock_file, 'w').close()
            with open(file_path, 'r') as file:
                task_string = file.read().strip()
            """
            從給定的字串中提取 task_name ,字串格式: {task_name}:{status},{x_pose},{y_pose}
            """
            # 初始化 x_pose 和 y_pose 的變數
            task_name = None

            # 分割 task_string 以提取座標
            if ':' in task_string and ',' in task_string:
                # 使用 ':' 分隔 task_name 和座標
                task_name, coordinates = task_string.split(':')
                status, x_pose, y_pose = coordinates.split(',')
                
            # 刪除假檔案標示檔案已關閉
            if os.path.exists(lock_file):
                os.remove(lock_file)
                
            return task_name, x_pose, y_pose

    def map_callback(self, msg):
        self.merge_map = msg.info
        self.map_explored = msg.data


    def generate_random_pose(self, explore_space):
        """
        生成隨機的 x_pose 和 y_pose,在 explore_space 範圍內。
        """
        count = 0
        while self.merge_map == None:
            self.subscription_map
            if count == 0:
                count += 1
                self.get_logger().info('Have not received map information yet.')
                self.get_logger().info('Waiting to receive map information...')
        
        if self.merge_map is not None:
            map_width = self.merge_map.width
            map_height = self.merge_map.height  # 獲取地圖的高度
            resolution = self.merge_map.resolution
            origin = self.merge_map.origin.position 
            x_max, y_max, x_min, y_min = explore_space

            # 取得未探索的格點
            unexplored_grid = []
            for y in range(map_height):
                for x in range(map_width):
                    index = y * map_width + x
                    if self.map_explored[index] == -1:
                        # 將未探索的格點座標加入到列表中
                        unexplored_grid.append((x, y))
            
            while True:
                # 取得未探索格點後進入無線迴圈(while ture)，從中隨機選取1個點轉換成真實座標 
                # ,並確認該點是在探索範圍內後回傳 xy 座標
                selected_coords = random.sample(unexplored_grid, 1)
                for coord in selected_coords:
                    x, y = coord
                # 將格點座標轉換為真實世界座標
                world_x = origin.x + (x + 0.5) * resolution
                world_y = origin.y + (y + 0.5) * resolution
                
                if (x_min <= world_x <= x_max) & (y_min <= world_y <= y_max):
                    return world_x, world_y


    def add_new_tasks(self, tasks_dict, explore_space, task_num):
        """
        在 tasks_dict 中生成新的任務，直到 'unexplored' 任務的數量達到 task_num
        """  
        new_task_count = task_num
        task_index = 0
        if tasks_dict != {}:
            unexplored_count = sum(1 for task in tasks_dict.values() if task['status'] == 'unexplored')
            new_task_count = task_num - unexplored_count
        else:
            new_task_count = task_num
            

        task_index = len(tasks_dict) + 1  # 從現有任務數量後開始新的任務命名
        for _ in range(new_task_count):
            x_pose, y_pose = self.generate_random_pose(explore_space)
            task_name = f"task_{task_index}"
            tasks_dict[task_name] = {
                'status': 'unexplored',
                'x_pose': x_pose,
                'y_pose': y_pose,
            }
            task_index += 1
        
        return tasks_dict

    def check_explored_task(self,tasks_dict):
        if self.merge_map is not None:
            map_width = self.merge_map.width
            map_height = self.merge_map.height  # 獲取地圖的高度
            resolution = self.merge_map.resolution
            origin = self.merge_map.origin.position 
            
            # 取得未探索的格點
            explored_grid = []
            count = 0
            for y in range(map_height):
                for x in range(map_width):
                    index = y * map_width + x
                    # 將已探索的格點座標加入到列表中
                    if self.map_explored[index] != -1:
                        explored_grid.append((x, y))
                        # if count == 0 :
                        #     self.get_logger().info(f'{explored_grid}')        
                        
            for task in tasks_dict.values() :
                if task['status'] == 'unexplored':
                    unexplored_task_x_real_coord = task['x_pose']
                    unexplored_task_y_real_coord = task['y_pose']
                    
                    # 將真實座標轉換成格點座標
                    task_x_pose = int(((unexplored_task_x_real_coord - origin.x) / resolution) - 0.5)
                    task_y_pose = int(((unexplored_task_y_real_coord - origin.y) / resolution) - 0.5)
                    gird_pose = (task_x_pose, task_y_pose)
                    if gird_pose in explored_grid :
                        self.get_logger().info(f'{task} has been explored.')
                        task['status'] = 'explored'
                        
            return tasks_dict
                        

    def update_complete_or_fail_task(self, tasks_dict):
        # 這段程式用來更新已經完成的任務或是無法完成的任務
        if os.path.exists(self.goal_fail_file):
            task_name, x_pose, y_pose = self.read_ongoing_fail_task(self.goal_fail_file)
            self.get_logger().info(f'Fail task file exist. {task_name} can not be explored.')
            tasks_dict[task_name]['status'] = 'unreachable' # 無法到達的任務目標會被標記為 unreachable
            os.remove(self.goal_fail_file)                  # 再更新完無法完成的任務後將 fail_task.txt 刪除

        tasks_dict = self.check_explored_task(tasks_dict)
        # task_name, x_pose, y_pose = self.read_ongoing_fail_task(self.ongoing_task_file_path)
        # tasks_dict[task_name]['status'] = 'explored'    # 成功到達的目標會被標記為被探索
        
        return tasks_dict
        
 
    def load_tasklist(self, file_path):
        # 確認檔案是否已被開啟的假檔案
        lock_file = file_path + '.lock'

        # 等待直到檔案被釋放
        if os.path.exists(lock_file):
            self.get_logger().info('Task array file is currently in use, waiting...')
            # time.sleep(0.1)  # 等待0.1秒後再檢查
        else:
            # 只在假檔案不存在時才執行,創建假檔案表示檔案被開啟
            open(lock_file, 'w').close() 
            
            with open(file_path, 'r') as file:
                task_string = file.read().strip()
            """
            解析任務字串並將其存入 dictionary 中
            格式為 {task_name}:{task_status},{x_pose},{y_pose};{task_name}:...
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
                    }
            
            # 刪除假檔案標示檔案已關閉
            if os.path.exists(lock_file):
                os.remove(lock_file)
        
        return tasks_dict
    

    def remove_old_file(self):
        # 此段程式是用來刪除 task_list.txt, best_assignment.txt  以及 database 中所有的 allocation_{robot_name}_successrate.txt
        robot_name_list = ['robot_A', 'robot_B', 'robot_C', 'robot_D', 'robot_E']
        if os.path.exists(self.task_list_file_path):
            self.get_logger().info('Remove old version task_list.txt')
            os.remove(self.task_list_file_path)
        
        # allocation_{robot_name}_successrate.txt
        for robot in robot_name_list:
            file_name = f'allocation_{robot}_successrate.txt'
            sr_file_path = os.path.join(self.database_path, file_name)
            if os.path.exists(sr_file_path):
                self.get_logger().info(f'Remove old version {file_name}')
                os.remove(sr_file_path)
            
    # done
    def save_task_list(self, new_tasks_dict):
        new_tasks_string = ''
        self.get_logger().info('Start combining new_tasks_string......')
        for task_name, task_details in new_tasks_dict.items():
            # self.get_logger().info(f'task_name : {task_name}')
            if new_tasks_string != '':
                new_tasks_string = new_tasks_string+';{}:{},{},{}'.format(  task_name,
                                                                            task_details['status'],
                                                                            task_details['x_pose'],
                                                                            task_details['y_pose'])
            elif new_tasks_string == '':
                new_tasks_string = new_tasks_string+'{}:{},{},{}'.format(   task_name,
                                                                            task_details['status'],
                                                                            task_details['x_pose'],
                                                                            task_details['y_pose'])
            
        self.get_logger().info('Finish combining new_tasks_string.')
        self.get_logger().info('Save task list file.')
        
        with open(self.task_list_file_path, 'w') as file:
            file.write(new_tasks_string)
            
        self.get_logger().info('Return new_tasks_string')
            
        return new_tasks_string


        
    def node_action(self, request, response):
        # 更新任務
        msg = request.send_msg
        self.get_logger().info(f'Recive message : {msg}')
        
        if os.path.exists(self.task_list_file_path):
            self.get_logger().info('Task list file exist. Loading task list ...')
            tasks_dict = self.load_tasklist(self.task_list_file_path)
            self.get_logger().info('Update task list ...')
            tasks_dict = self.update_complete_or_fail_task(tasks_dict)
        else:
            self.get_logger().info('Task list file is not exist. Create new task list ...')
            tasks_dict = {}
            
        new_tasks_dict = self.add_new_tasks(tasks_dict, self.explore_space, self.task_num)
        self.remove_old_file()
        new_tasks_string = self.save_task_list(new_tasks_dict)
        self.get_logger().info('Send back new_tasks_string')
        response.new_task_list = new_tasks_string
        
        return response



def main(args=None):
    rclpy.init(args=args)
    agent = TaskAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
