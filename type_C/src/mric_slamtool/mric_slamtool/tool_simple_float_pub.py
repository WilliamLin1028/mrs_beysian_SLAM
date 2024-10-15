import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

class PubFloat(Node):
    def __init__(self):
        super().__init__('tool_pub_simple_float')

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.node_action)
        
        # self.pub_workstatus = self.create_publisher(String, '/mric_simple_publisher', 10)
        self.pub = self.create_publisher(String, '/battery', 10)
        
    def publish_msg(self, msg):
        self.pub.publish(msg)

    
    def node_action(self):
        # 先發佈機器人工作狀態
        msg = Float32()
        msg.data = 11.4
        self.publish_msg(msg)
        
        
def main():
    rclpy.init()
    agent = PubFloat()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()