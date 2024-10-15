import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory

import random, os, time

'''
該程式在rviz上加入了任務座標點,並且該點帶有任務名稱的標記
'''


class PointPublisher(Node):
    def __init__(self):
        super().__init__('point_publisher')
        bringup_dir = get_package_share_directory('mric_slamtool')
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.task_array_txt = os.path.join(bringup_dir, 'database', 'task_list.txt')
        self.timer = self.create_timer(3.0, self.timer_callback)
    
    def load_taskarray(self):
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
                    }
            
            # 刪除假檔案標示檔案已關閉
            if os.path.exists(lock_file):
                os.remove(lock_file)

        return tasks_dict
        

    def timer_callback(self):
        if os.path.exists(self.task_array_txt):
            marker_array = MarkerArray()
            tasks_dict = self.load_taskarray()                              # 載入 task list 並將 string 轉換成 dict
            
            # 創建點標記
            # marker = Marker()
            # marker.header.frame_id = "map"
            # marker.header.stamp = self.get_clock().now().to_msg()
            # marker.ns = "points"
            # marker.id = i
            # marker.type = Marker.SPHERE
            # marker.action = Marker.ADD
            i = 0

            for task_name, task_details in tasks_dict.items():
                if tasks_dict[task_name]['status'] != 'completed':
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
                    
                    marker.scale.x = 0.2
                    marker.scale.y = 0.2
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
                    
                    text_marker.scale.z = 0.2  # 設置文字大小
                    
                    text_marker.color.a = 1.0  # 透明度
                    text_marker.color.r = 0.0
                    text_marker.color.g = 1.0
                    text_marker.color.b = 0.0
                    
                    text_marker.text = "Point_{}".format(i+1)  # 設置文字內容
                    
                    marker_array.markers.append(text_marker)
                    i +=1 

            
            self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
