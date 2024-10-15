import rclpy
import os

from rclpy.node import Node

from mric_slamtool_interfaces.srv import ConnectReciver, UpdateTaskList, AllocateTask, RequestSuccessrate, SendNewTask

from ament_index_python.packages import get_package_share_directory


'''
此程式用來在 server 外接收 auctioneer 發送的新 task_list 並更新,等待 inside_successrate_agent.py 計算新 task_list 的所有任務成功率
,並將新的 task success rate 回傳給 auctioneer 的 allocation_agent
'''
robot_name = 'robot_A'    # 此機器人的名稱

class SuccessrateRequestAgent(Node):
    def __init__(self):
        super().__init__(f'{robot_name}_sr_request_reciver_agent')
        self.srv = self.create_service(RequestSuccessrate, f'{robot_name}_successrate_cal', self.node_action)

        bringup_dir = get_package_share_directory('mric_slamtool')
        self.task_list_txt = os.path.join(bringup_dir, 'database', 'task_list.txt')
        self.successrate_txt = os.path.join(bringup_dir, 'database', 'successrate.txt')
        self.cmd_request_sr_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_request_successrate.txt')

    
    def node_action(self, request, response):
        new_task_list = request.task_list
        # 格式 : task_1:unexplored,0.875000162050128,-0.47499985806643963
        self.get_logger().info('Recive task_list : {}'.format(new_task_list))
        
        if os.path.exists(self.successrate_txt):
            self.get_logger().info('Remove old version successrate.txt')
            os.remove(self.successrate_txt)
        
        if os.path.exists(self.task_list_txt):
            self.get_logger().info('Remove old version task_list.txt')
            os.remove(self.task_list_txt)
            
        self.get_logger().info('Create new version task_list.txt')
        with open(self.task_list_txt, 'w') as file:
            file.write(new_task_list)
        
        self.get_logger().info('Create cmd_request_successrate.txt to trigger on inside_successrate_agent.py calculate task list success rate ...')
        open(self.cmd_request_sr_file_path, 'w').close()
        
        count = 0
        while os.path.exists(self.cmd_request_sr_file_path):
            if os.path.exists(self.successrate_txt):
                self.get_logger().info('successrate.txt is exist, task list success rate has been calculated.')
                os.remove(self.cmd_request_sr_file_path)
            else:
                if count == 0:
                    count += 1
                    self.get_logger().info('Wait for inside_successrate_agent.py calculate success rate ...')
    
        with open(self.successrate_txt, 'r') as file:
            sr_string = file.read().strip()
                
        response.successrate_string = sr_string
        
        return response 
            


def main(args=None):
    rclpy.init(args=args)
    agent = SuccessrateRequestAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()