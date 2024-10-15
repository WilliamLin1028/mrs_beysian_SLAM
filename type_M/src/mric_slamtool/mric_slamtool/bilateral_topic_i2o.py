import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

'''
此程式用來將接收到的 topic publish 訊息轉換成新的名稱,並重新發佈出去
可搭配 fastdds 將無法修改名稱的 node 將訊息轉發或接收
'''

robot_name = 'robot_D'

class I2O_Topic_Bridge(Node):
    def __init__(self):
        super().__init__(f'{robot_name}_i2o_bridge_agent')

        self.show_termial_msg = False       # 若為 Ture 表示要在 termial 顯示訊息

        # in >> out
        self.local_map_sub = self.create_subscription(OccupancyGrid, '/map', self.robot_local_map_callback, 10)
        self.local_map_repub = self.create_publisher(OccupancyGrid, f'/{robot_name}_map', 10)

        # in >> out
        # self.work_status_sub = self.create_subscription(String, '/workstatus', self.robot_work_status_callback, 10)
        # self.work_status_repub = self.create_publisher(String, f'/{robot_name}_workstatus', 10)



    def termial_msg(self, head_str, pub_string):
        if self.show_termial_msg == True:
            str_1 = f'\n========================== [{head_str}] =========================='
            str_2 = f'\n{pub_string}'
            str_3 = '\n==================================================================='
            total_str = str_1 + str_2 + str_3
            self.get_logger().info(total_str)
    
    ################################## local map topic convert ##################################
    def robot_local_map_callback(self, msg):
        head_str = 'Convert local map'
        str_1 = f'\n Recive {robot_name} local map.'
        str_2 = f'\n Topic /map convert to /{robot_name}_map.'
        str_3 = f'\n Publish msg to /{robot_name}_map.'
        pub_str = str_1 + str_2 + str_3
        self.robot_local_map_publish(msg)
        self.termial_msg(head_str, pub_str)
    
    def robot_local_map_publish(self, msg):
        self.local_map_repub.publish(msg)


    ################################## work status topic convert ##################################
    def robot_work_status_callback(self, msg):
        head_str = 'Convert work_status'
        str_1 = f'\n Recive {robot_name} work status.'
        str_2 = f'\n Topic /workstatus convert to /{robot_name}_work_status.'
        str_3 = f'\n Publish msg to /{robot_name}_work_status.'
        pub_str = str_1 + str_2 + str_3
        self.robot_work_status_publish(msg)
        self.termial_msg(head_str, pub_str)
    
    def robot_work_status_publish(self, msg):
        self.work_status_repub.publish(msg)
        

def main():
    rclpy.init()
    agent = I2O_Topic_Bridge()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()