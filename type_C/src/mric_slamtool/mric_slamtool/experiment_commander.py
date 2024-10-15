import rclpy
from rclpy.node import Node

from std_msgs.msg import String
# from mric_slamtool_interfaces.srv import StartExperiment

#attend_robot = ['robot_A', 'robot_B', 'robot_C', 'robot_D', 'robot_E']
attend_robot = ['robot_A']

class ExperimentCommander(Node):
    def __init__(self):
        super().__init__('experiment_commander_node')
        
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.node_action)
        self.pub_robot_map = self.create_publisher(String, '/start_experiment', 10)

    def publish_experiment_start_msg(self, msg):
        # 如果地圖資訊不為空，則處理任務
        self.pub_robot_map.publish(msg)

    def node_action(self):
        attend_string = ''
        
        for i in range(len(attend_robot)):
            if attend_string == '':
                attend_string = attend_robot[i]
            else:
                attend_string = attend_string +','+ attend_robot[i]
        
        self.get_logger().info('These robots attend this experiment : {}'.format(attend_string))
        
        self.get_logger().info('Send start msg.')
        start_experiment =String()
        
        start_experiment.data = 'Start Experiment;{}'.format(attend_string)
        self.publish_experiment_start_msg(start_experiment)



def main():
    rclpy.init()
    agent = ExperimentCommander()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()