# Authors: Abdulkadir Ture
# Github : abdulkadrtr

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import OccupancyGrid
import numpy as np
import yaml
import os

from ament_index_python.packages import get_package_share_directory

robot_name = 'robot_A'

def merge_maps(map1, map2):
    # map1 作為輸入 map , map2 作為暨存 map , 若 map2 為空，則 map2 = map1
    if map2 == None:
        map2 = map1
    
    merged_map = OccupancyGrid()
    merged_map.header = map1.header
    merged_map.header.frame_id = 'map'
    min_x = min(map1.info.origin.position.x, map2.info.origin.position.x)
    min_y = min(map1.info.origin.position.y, map2.info.origin.position.y)
    max_x = max(map1.info.origin.position.x + (map1.info.width * map1.info.resolution),
                map2.info.origin.position.x + (map2.info.width * map2.info.resolution))
    max_y = max(map1.info.origin.position.y + (map1.info.height * map1.info.resolution),
                map2.info.origin.position.y + (map2.info.height * map2.info.resolution))
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.resolution = min(map1.info.resolution, map2.info.resolution)
    merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
    merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)
    for y in range(map1.info.height):
        for x in range(map1.info.width):
            i = x + y * map1.info.width
            merged_x = int(np.floor((map1.info.origin.position.x + x * map1.info.resolution - min_x) / merged_map.info.resolution))
            merged_y = int(np.floor((map1.info.origin.position.y + y * map1.info.resolution - min_y) / merged_map.info.resolution))
            merged_i = merged_x + merged_y * merged_map.info.width
            merged_map.data[merged_i] = map1.data[i]
    for y in range(map2.info.height):
        for x in range(map2.info.width):
            i = x + y * map2.info.width
            merged_x = int(np.floor((map2.info.origin.position.x + x * map2.info.resolution - min_x) / merged_map.info.resolution))
            merged_y = int(np.floor((map2.info.origin.position.y + y * map2.info.resolution - min_y) / merged_map.info.resolution))
            merged_i = merged_x + merged_y * merged_map.info.width
            if merged_map.data[merged_i] == -1:
                merged_map.data[merged_i] = map2.data[i]
    return merged_map


class MergeMapNode(Node):
    def __init__(self):
        super().__init__(f'{robot_name}_merge_map_node')
        #qos = QoSProfile(depth=10, reliability=QoSProfile.ReliabilityPolicy.RELIABLE, durability=QoSProfile.DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, f'/{robot_name}_merge_map', 10)
        # self.publisher = self.create_publisher(OccupancyGrid, '/map', qos)
        self.subscription = self.create_subscription(OccupancyGrid, f'/{robot_name}_map', self.map1_callback, 10)
        self.subscription = self.create_subscription(OccupancyGrid, f'/robot_D_map', self.map2_callback, 10)
        
        bringup_dir = get_package_share_directory('mric_slamtool')
        self.mergemap_info_file_path = os.path.join(bringup_dir, 'database', 'merge_map_info.txt')
        self.mergemap_data_file_path = os.path.join(bringup_dir, 'database', 'merge_map_data.txt')
        
        self.map1 = None
        self.map1_rec_count = 0
        self.map2 = None
        self.switch_load = 'map_2_load'
        self.merge_map =None
        self.termialmsg_switch = True       # True表示不會在termial上發布狀態資訊

    def map1_callback(self, msg):
        if self.map1_rec_count == 20 :
            self.get_logger().info(f'Recive {robot_name}_map.')
            self.map1_rec_count = 0
        self.map1 = msg
        self.map1_rec_count +=1
        #if self.switch_load == 'map_2_load':
        # if self.map2 is not None:
        #     self.map2.info.origin.position.x = self.map2.info.origin.position.x
        #     self.map2.info.origin.position.y = self.map2.info.origin.position.y
        #     msg = merge_maps(self.map1, self.map2)
        #     # self.save_string_to_file(self.mergemap_info_file_path, self.mergemap_data_file_path, msg)
        #     self.publisher.publish(msg)
        self.merge_map = merge_maps(self.map1, self.merge_map)
        self.publisher.publish(self.merge_map)
        self.switch_load = 'map_1_load'
        
    def map2_callback(self, msg):
        self.get_logger().info(f'Recive robot_D_map.')
        self.map2 = msg
        #if self.switch_load == 'map_1_load':
        # if self.map1 is not None:
        #     self.map2.info.origin.position.x = self.map2.info.origin.position.x
        #     self.map2.info.origin.position.y = self.map2.info.origin.position.y
        #     msg = merge_maps(self.map1, self.map2)
        #     # self.save_string_to_file(self.mergemap_info_file_path, self.mergemap_data_file_path, msg)
        #     self.publisher.publish(msg)
        self.merge_map = merge_maps(self.map2, self.merge_map)
        self.publisher.publish(self.merge_map)
        self.switch_load = 'map_2_load'
    
    def termial_msg(self, msg_text):
        if self.msg_text != True:
            self.get_logger().info(msg_text)
        
            
    def save_string_to_file(self, info_path, data_file, msg):
        """
        將字串資料存入txt檔案,如果該檔案不存在,則新增該檔案
        """
        self.get_logger().info('frame_id : {}'.format(msg.header.frame_id))
        self.get_logger().info('width : {}'.format(msg.info.width))
        self.get_logger().info('height : {}'.format(msg.info.height))
        self.get_logger().info('resolution : {}'.format(msg.info.resolution))
        self.get_logger().info('origin_x : {}'.format(msg.info.origin.position.x))
        self.get_logger().info('origin_y : {}'.format(msg.info.origin.position.y))
        info_data = {
            'frame_id': msg.header.frame_id,
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y
            }
        
        map_data = []
        map_data = msg.data
        map_data = np.array(map_data, dtype=np.int8)
        
        try:
            # 寫入資料到檔案，若檔案不存在則會自動建立
            with open(info_path, 'w') as file:
                yaml.dump(info_data, file)
            self.get_logger().info(f'Info Data has been written to {info_path}')
        
            np.savetxt(data_file, map_data)
            self.get_logger().info(f'Map Data has been written to {data_file}')
        
        except Exception as e:
            self.get_logger().info(f'An error occurred: {e}')

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
