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
robot_name = 'robot_A'

class PubMergemap_InsServer(Node):
    def __init__(self):
        super().__init__(f'pub_{robot_name}_map_agent')
        bringup_dir = get_package_share_directory('mric_slamtool')
        
        self.robot_standby_file_path = os.path.join(bringup_dir, 'cmd_folder', 'robot_standby.txt')
        self.robot_ongoing_file_path = os.path.join(bringup_dir, 'cmd_folder', 'robot_ongoing.txt')
        
        self.mergemap_info_file_path = os.path.join(bringup_dir, 'database', 'robot_map_info.txt')
        self.mergemap_data_file_path = os.path.join(bringup_dir, 'database', 'robot_map_data.txt')
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.node_action)
        self.pub_robot_map = self.create_publisher(OccupancyGrid, f'/{robot_name}_map', 10)
        self.pub_workstatus = self.create_publisher(String, f'/{robot_name}_workstatus', 10)
        
    def publish_map_msg(self, msg):
        # 如果地圖資訊不為空，則處理任務
        self.pub_robot_map.publish(msg)
        
    def publish_workstatus_msg(self, msg):
        self.pub_workstatus.publish(msg)
    
    def check_workstatus_file(self):
        standby = os.path.isfile(self.robot_standby_file_path)
        ongoing = os.path.isfile(self.robot_ongoing_file_path)
        if standby == True & ongoing == False :
            self.get_logger().info('Robot status : standby')
            return 'standby'
        elif standby == False & ongoing == True :
            self.get_logger().info('Robot status : ongoing')
            return 'ongoing'
        else :
            self.get_logger().info('Robot status : error')
            return
    
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
        # 先發佈機器人工作狀態
        workstatus_msg = self.check_file_existence()
        self.publish_map_msg(workstatus_msg)
        
        # 確認是否已經存在該檔案
        existence_message = self.check_file_existence(self.mergemap_info_file_path, self.mergemap_data_file_path)
        self.get_logger().info(existence_message)
        # 若存在,則確認是否被開啟
        if existence_message == 'file is exist':
            map_lock_file = self.mergemap_data_file_path + '.lock'
            info_lock_file = self.mergemap_info_file_path + '.lock'
            # 檢查暫存檔案是否存在
            if os.path.isfile(map_lock_file) or os.path.isfile(info_lock_file):
                self.get_logger().info(f'Lock file already exists. Operation aborted.')
                return

            try:
                self.get_logger().info(f'Lock file is not exists. Start loading map file, build Lock file.')
                # 建立暫存檔案
                open(map_lock_file, 'w').close()
                open(info_lock_file, 'w').close()
                
                # info_file_path = os.path.join(self.file_dict, self.info_file_name)
                
                info_f = open(self.mergemap_info_file_path, 'r', encoding='utf-8')
                robot_config = yaml.load(info_f.read(), Loader=yaml.FullLoader)
                # self.get_logger().info('{}'.format(f))
                info_f.close()
                
                # map_file_path = os.path.join(self.file_dict, self.map_file_name)
                map_data = np.loadtxt(self.mergemap_data_file_path)
                map_data_int = map_data.astype('int8')
                # self.get_logger().info('{}'.format(map_data_int))
                #map_data = np.array(map_data, dtype=np.int8)
                # map_f = open(map_file_path, 'r', encoding='utf-8')
                # map_data = map_f.read()
                # map_f.close()
                
                merged_map = OccupancyGrid()
                
                self.get_logger().info(f'Loading map config......')
                
                for key, value in robot_config.items():
                    #self.get_logger().info('key : {}'.format(key))
                    if key == 'frame_id':
                        self.get_logger().info('frame_id : {}'.format(value))
                        merged_map.header.frame_id = '{}'.format(value)
                    elif key == 'width':
                        self.get_logger().info('width : {}'.format(value))
                        #self.get_logger().info('type width : {}'.format(type(value)))
                        d = int(value)
                        merged_map.info.width = d
                    elif key == 'height':
                        self.get_logger().info('height : {}'.format(value))
                        #self.get_logger().info('type height : {}'.format(type(value)))
                        d = int(value)
                        merged_map.info.height = d
                    elif key == 'resolution':
                        self.get_logger().info('resolution : {}'.format(value))
                        #self.get_logger().info('type resolution : {}'.format(type(value)))
                        d = float(value)
                        merged_map.info.resolution = d
                    elif key == 'origin_x':
                        self.get_logger().info('origin_x : {}'.format(value))
                        #self.get_logger().info('type origin_x : {}'.format(type(value)))
                        d = float(value)
                        merged_map.info.origin.position.x = d
                    elif key == 'origin_y':
                        self.get_logger().info('origin_y : {}'.format(value))
                        #self.get_logger().info('type origin_y : {}'.format(type(value)))
                        d = float(value)
                        merged_map.info.origin.position.y = d
                        
                self.get_logger().info(f'Loading grid map......')
                merged_map.data = [-1] * len(map_data_int)
                # for y in range(merged_map.info.height):
                #     for x in range(merged_map.info.width):
                #         i = x + y * merged_map.info.width
                #         merged_x = int(np.floor(merged_map.info.origin.position.x / merged_map.info.resolution))
                #         merged_y = int(np.floor(merged_map.info.origin.position.y / merged_map.info.resolution))
                #         merged_i = merged_x + merged_y * merged_map.info.width
                #         merged_map.data[merged_i] = merged_map.data[i]
                for i in range(len(map_data_int)):
                    merged_map.data[i] = map_data_int[i]
                self.get_logger().info(f'Success load Grid map.')
                self.publish_map_msg(merged_map)
            
            except FileNotFoundError:
                self.get_logger().info('file is not exist')
            
            finally:
                # 刪除暫存檔案
                self.get_logger().info(f'Finish loading map file, remove temporary file.')
                if os.path.isfile(info_lock_file):
                    os.remove(info_lock_file)
                if os.path.isfile(map_lock_file):
                    os.remove(map_lock_file)
            
        # 若不存在直接儲存字串
        else:
            self.get_logger().info('file {} is not exist'.format(self.map_file_name))

    # def data2topic(self, file_data)
            
def main():
    rclpy.init()
    agent = PubMergemap_InsServer()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()