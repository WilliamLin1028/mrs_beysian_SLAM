import rclpy
import os

from rclpy.node import Node
# from rclpy.action import ActionServer
# from mric_action.action import RequestSuccessrate

from mric_slamtool_interfaces.srv import ConnectReciver, UpdateTaskList, AllocateTask, RequestSuccessrate, SendNewTask

from ament_index_python.packages import get_package_share_directory


'''
此程式用來在 server 外接收 auctioneer 發送的新任務
'''
robot_name = 'robot_A'    # 此機器人的名稱

class NewTaskReciverAgent(Node):
    def __init__(self):
        super().__init__(f'{robot_name}_newtask_reciver_agent')
        self.srv = self.create_service(SendNewTask, f'{robot_name}_newtask', self.node_action)
        
        self.recive_task_name = None
        self.ongoing_task_name = None
        
        bringup_dir = get_package_share_directory('mric_slamtool')
        self.ongoing_task_file_path = os.path.join(bringup_dir, 'database', 'ongoing_task.txt')
        self.cmd_update_ongoing_task_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_update_ongoing_task.txt')

    
    def node_action(self, request, response):
        new_task_str = request.new_task_string
        # 格式 : task_1:unexplored,0.875000162050128,-0.47499985806643963
        self.get_logger().info('Recive new task : {}'.format(new_task_str))
        
        if self.ongoing_task_name != new_task_str:
            self.get_logger().info('New task is different with old task : {}'.format(self.ongoing_task_name))
            self.get_logger().info('Update new task ......')
            self.ongoing_task_name = new_task_str
            
            if os.path.exists(self.ongoing_task_file_path):
                self.get_logger().info('Remove old task ......')
                os.remove(self.ongoing_task_file_path)
                
            with open(self.ongoing_task_file_path, 'w') as file:
                file.write(new_task_str)
                
            open(self.cmd_update_ongoing_task_file_path, 'w').close()
            
            count = 0
            while os.path.exists(self.cmd_update_ongoing_task_file_path):
                if count == 0 :
                    self.get_logger().info('Waiting inside_executor update task ......')
                    count += 1
            
        else:
            self.get_logger().info('New task is same to old.')
        
        self.get_logger().info('Task has been updated, send the feedback to allocation agent.')
        response.feedback_msg = 'Recive new task'
        
        return response 


def main(args=None):
    rclpy.init(args=args)
    agent = NewTaskReciverAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()