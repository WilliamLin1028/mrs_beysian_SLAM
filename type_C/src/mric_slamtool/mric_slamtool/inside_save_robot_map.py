import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import yaml
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory


class SaveRobotMap_InServer(Node):
    '''
    這個檔案是用來將儲存機器人所掃描到的localmap存入database
    ,以供不同機器人的地圖分享與合併
    '''
    def __init__(self):
        super().__init__('save_robotmap_agent')
        bringup_dir = get_package_share_directory('mric_slamtool')
        
        self.map_info_file_path = os.path.join(bringup_dir, 'database', 'robot_map_info.yaml')
        self.map_data_file_path = os.path.join(bringup_dir, 'database', 'robot_map_data.txt')
        self.map_msg = None
        
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.node_action)
        self.i = 0
        self.msg_text = True
        self.termial_publish_count = 10         # 在累計接收到一定數量的publisher資訊後才會在termial上發布訊息

    def termial_msg(self, msg_text):
        if self.msg_text != True:
            self.get_logger().info(msg_text)
        
    def map_callback(self, msg):
        # 如果地圖資訊不為空，則處理任務
        if msg.data != None :
            # msg2str = '{}'.format(msg.data)
            if self.termial_publish_count >= 10:
                self.get_logger().info('Recive grid map')
            self.map_msg = msg
            # self.node_action(self.map_info_file_path,  self.map_data_file_path, msg)
    
    def check_file_existence(self, info_file_path, map_file_path):
        """
        檢查指定目錄內是否存在指定檔案
        參數:   directory (str): 要檢查的資料夾路徑
                filename (str): 要檢查的檔案名稱
        回傳:   str: 檔案是否存在的訊息
        """
        # 檢查檔案是否存在
        if os.path.isfile(info_file_path) & os.path.isfile(map_file_path):
            return 'file is exist'
        else:
            return 'file is not exist'
            
    def node_action(self):
        info_path = self.map_info_file_path
        data_file = self.map_data_file_path
        msg = self.map_msg
        
        existence_message = self.check_file_existence(info_path, data_file)
        self.get_logger().info(existence_message)
        # 若存在,則確認是否被開啟
        if existence_message == 'file is exist':
            map_lock_file = data_file + '.lock'
            info_lock_file = info_path + '.lock'
            # 檢查暫存檔案是否存在
            if os.path.isfile(map_lock_file) or os.path.isfile(info_lock_file):
                self.get_logger().info(f'Lock file already exists. Operation aborted.')
                return

            try:
                self.get_logger().info(f'Lock file is not exists. Start loading map file, build Lock file.')
                # 建立暫存檔案
                open(map_lock_file, 'w').close()
                open(info_lock_file, 'w').close()
                
                if os.path.isfile(data_file):
                    os.remove(data_file)
                if os.path.isfile(info_path):
                    os.remove(info_path)
                    
                if self.map_msg != None :
                    self.save_string_to_file(info_path, data_file, msg)
            
            except FileNotFoundError:
                self.get_logger().info('file is not exist')
            
            finally:
                # 刪除暫存檔案
                if os.path.isfile(map_lock_file):
                    os.remove(map_lock_file)
                if os.path.isfile(info_lock_file):
                    os.remove(info_lock_file)
            
        # 若不存在直接儲存字串
        else:
            if self.map_msg != None :
                self.save_string_to_file(info_path, data_file, msg)
                
    def save_string_to_file(self, info_path, data_file, msg):
        """
        將字串資料存入txt檔案,如果該檔案不存在,則新增該檔案
        """
        # self.get_logger().info(msg)
        if self.termial_publish_count >= 10:
            self.get_logger().info('========== save local map ==========')
            self.get_logger().info('frame_id : {}'.format(msg.header.frame_id))
            self.get_logger().info('width : {}'.format(msg.info.width))
            self.get_logger().info('height : {}'.format(msg.info.height))
            self.get_logger().info('resolution : {}'.format(msg.info.resolution))
            self.get_logger().info('origin_x : {}'.format(msg.info.origin.position.x))
            self.get_logger().info('origin_y : {}'.format(msg.info.origin.position.y))
            self.get_logger().info('====================================')
            self.termial_publish_count = 0
        
        info_data = {
            'frame_id': msg.header.frame_id,
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y
            }
        
        self.termial_publish_count += 1
        
        map_data = []
        map_data = msg.data
        map_data = np.array(map_data, dtype=np.int8)
        
        try:
            # 寫入資料到檔案，若檔案不存在則會自動建立
            with open(info_path, 'w') as file:
                yaml.dump(info_data, file)
            #self.get_logger().info(f'Info Data has been written to {info_path}')
        
            np.savetxt(data_file, map_data)
            #self.get_logger().info(f'Map Data has been written to {data_file}')
        
        except Exception as e:
            self.get_logger().info(f'An error occurred: {e}')
            
def main():
    rclpy.init()
    agent = SaveRobotMap_InServer()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()