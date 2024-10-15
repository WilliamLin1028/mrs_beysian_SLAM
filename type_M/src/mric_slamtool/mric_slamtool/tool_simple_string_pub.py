import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PubString(Node):
    def __init__(self):
        super().__init__('tool_pub_simple_string')

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.node_action)
        
        # self.pub_workstatus = self.create_publisher(String, '/mric_simple_publisher', 10)
        self.pub_workstatus = self.create_publisher(String, '/ongoing_task', 10)
        
    def publish_msg(self, msg):
        self.pub_workstatus.publish(msg)

    
    def node_action(self):
        # 先發佈機器人工作狀態
        msg = String()
        msg.data = 'task_1:unexplored,0.875000162050128,-0.47499985806643963'
        self.publish_msg(msg)
        
        
def main():
    rclpy.init()
    agent = PubString()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()