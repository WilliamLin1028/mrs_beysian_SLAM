import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import yaml
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

'''
這個程式是透過讀取 server 內掃描到的 map.txt 檔,重新在 server 外發布的程式
'''
robot_name = 'robot_B'

class PubMergeMap(Node):
    def __init__(self):
        super().__init__('pub_merge_map_agent')
        bringup_dir = get_package_share_directory('mric_slamtool')
        
        self.mergemap_info_file_path = os.path.join(bringup_dir, 'database', 'merge_map_info.yaml')
        self.mergemap_data_file_path = os.path.join(bringup_dir, 'database', 'merge_map_data.txt')
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.node_action)
        self.pub_robot_map = self.create_publisher(OccupancyGrid, '/merge_map', 10)
        self.termial_publish_count = 50         # 在累計接收到一定數量的publisher資訊後才會在termial上發布訊息
        
        
    def publish_map_msg(self, msg):
        # 如果地圖資訊不為空，則處理任務
        self.pub_robot_map.publish(msg)
        
    
    def check_map_file(self, info_file_path, map_file_path):
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
    
    def termial_pub_check(self, msg):
        if self.termial_publish_count >= 300:
            self.get_logger().info(msg)
    
    def node_action(self):
        # 確認是否已經存在該檔案
        existence_message = self.check_map_file(self.mergemap_info_file_path, self.mergemap_data_file_path)
        if self.termial_publish_count %10 == 0 :
            self.get_logger().info(existence_message)
        # 若存在,則確認是否被開啟
        if existence_message == 'file is exist':
            map_lock_file = self.mergemap_data_file_path + '.lock'
            info_lock_file = self.mergemap_info_file_path + '.lock'
            # 檢查暫存檔案是否存在
            if os.path.isfile(map_lock_file) or os.path.isfile(info_lock_file):
                if self.termial_publish_count %10 == 0 :
                    self.get_logger().info(f'Lock file already exists. Operation aborted.')
                return

            try:
                self.termial_pub_check('========== pub merge map ==========')
                self.termial_pub_check(f'Lock file is not exists. Start loading map file, build Lock file.')
                # self.get_logger().info(f'Lock file is not exists. Start loading map file, build Lock file.')
                # 建立暫存檔案
                open(map_lock_file, 'w').close()
                open(info_lock_file, 'w').close()
                
                info_f = open(self.mergemap_info_file_path, 'r', encoding='utf-8')
                robot_config = yaml.load(info_f.read(), Loader=yaml.FullLoader)
                info_f.close()
                
                map_data = np.loadtxt(self.mergemap_data_file_path)
                map_data_int = map_data.astype('int8')
                
                local_map = OccupancyGrid()
                
                self.termial_pub_check(f'Loading map config......')
                
                for key, value in robot_config.items():
                    #self.get_logger().info('key : {}'.format(key))
                    if key == 'frame_id':
                        self.termial_pub_check('frame_id : {}'.format(value))
                        #self.get_logger().info('frame_id : {}'.format(value))
                        local_map.header.frame_id = '{}'.format(value)
                    elif key == 'width':
                        self.termial_pub_check('width : {}'.format(value))
                        #self.get_logger().info('width : {}'.format(value))
                        d = int(value)
                        local_map.info.width = d
                    elif key == 'height':
                        self.termial_pub_check('height : {}'.format(value))
                        #self.get_logger().info('height : {}'.format(value))
                        d = int(value)
                        local_map.info.height = d
                    elif key == 'resolution':
                        self.termial_pub_check('resolution : {}'.format(value))
                        #self.get_logger().info('resolution : {}'.format(value))
                        d = float(value)
                        local_map.info.resolution = d
                    elif key == 'origin_x':
                        self.termial_pub_check('origin_x : {}'.format(value))
                        #self.get_logger().info('origin_x : {}'.format(value))
                        d = float(value)
                        local_map.info.origin.position.x = d
                    elif key == 'origin_y':
                        self.termial_pub_check('origin_y : {}'.format(value))
                        #self.get_logger().info('origin_y : {}'.format(value))
                        d = float(value)
                        local_map.info.origin.position.y = d
                
                        
                self.termial_pub_check(f'Loading grid map......')
                #self.get_logger().info(f'Loading grid map......')
                local_map.data = [-1] * len(map_data_int)
                for i in range(len(map_data_int)):
                    local_map.data[i] = map_data_int[i]
                self.termial_pub_check(f'Success load Grid map.')
                #self.get_logger().info(f'Success load Grid map.')
                self.publish_map_msg(local_map)
                
                self.termial_publish_count +=1
                self.termial_pub_check('====================================')
                if self.termial_publish_count > 300:
                    self.termial_publish_count = 0
            
            except FileNotFoundError:
                self.get_logger().info('file is not exist')
            
            finally:
                # 刪除暫存檔案
                if self.termial_publish_count %20 == 0 :
                    self.get_logger().info(f'Finish loading map file, remove temporary file.')
                if os.path.isfile(info_lock_file):
                    os.remove(info_lock_file)
                if os.path.isfile(map_lock_file):
                    os.remove(map_lock_file)
        else:
            if self.termial_publish_count %10 == 0 :
                self.get_logger().info('Wait for a second...')

    # def data2topic(self, file_data)
            
def main():
    rclpy.init()
    agent = PubMergeMap()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()