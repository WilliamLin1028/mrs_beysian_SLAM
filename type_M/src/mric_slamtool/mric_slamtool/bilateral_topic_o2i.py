import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

'''
此程式用來將接收到的 topic publish 訊息轉換成新的名稱,並重新發佈出去
可搭配 fastdds 將無法修改名稱的 node 將訊息轉發或接收
'''

robot_name = 'robot_D'

class O2I_Topic_Bridge(Node):
    def __init__(self):
        super().__init__(f'{robot_name}_o2i_bridge_agent')

        self.show_termial_msg = False       # 若為 Ture 表示要在 termial 顯示訊息

        # out >> in
        self.merge_map_sub = self.create_subscription(OccupancyGrid, f'/{robot_name}_merge_map', self.robot_merge_map_callback, 10)
        self.merge_map_repub = self.create_publisher(OccupancyGrid, f'/merge_map', 10)



    def termial_msg(self, head_str, pub_string):
        if self.show_termial_msg == True:
            str_1 = f'\n========================== [{head_str}] =========================='
            str_2 = f'\n{pub_string}'
            str_3 = '\n==================================================================='
            total_str = str_1 + str_2 + str_3
            self.get_logger().info(total_str)

    ################################## merge map topic convert ##################################
    def robot_merge_map_callback(self, msg):
        head_str = 'Convert merge map'
        str_1 = f'\n Recive {robot_name} merge map.'
        str_2 = f'\n Topic /{robot_name}_map convert to /merge_map.'
        str_3 = f'\n Publish msg to /merge_map.'
        pub_str = str_1 + str_2 + str_3
        self.robot_merge_map_publish(msg)
        self.termial_msg(head_str, pub_str)
    
    def robot_merge_map_publish(self, msg):
        self.merge_map_repub.publish(msg)
        

def main():
    rclpy.init()
    agent = O2I_Topic_Bridge()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()