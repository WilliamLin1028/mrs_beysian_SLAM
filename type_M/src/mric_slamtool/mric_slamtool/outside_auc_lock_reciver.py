import rclpy
import os

from rclpy.node import Node
# from rclpy.action import ActionServer
# from mric_action.action import RequestSuccessrate

from mric_slamtool_interfaces.srv import ConnectReciver, LockAuctioneer
from ament_index_python.packages import get_package_share_directory


robot_name = 'robot_D'    # 此機器人的名稱

class AuctioneerLockerAgent(Node):
    def __init__(self):
        super().__init__(f'{robot_name}_auctioneer_lock_reciver_node')
        self.srv = self.create_service(LockAuctioneer, f'{robot_name}_lock_reciver', self.node_action)
        
        bringup_dir = get_package_share_directory('mric_slamtool')
        self.auctioneerlocker_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_wait_for_auctioneer.txt')
    
    def node_action(self, request, response):
        '''
        service 的訊息 request 類型有三種,分別是
        "分配機鎖定與否的 allocation_lock",其內容為 : 
        "儲存 task list 並用於要求計算成功率的 tasks_list",其內容為 : {task_name}:{task_status},{x_pose},{y_pose};{task_name}:...
        "儲存新任務的 assigned_task",其內容為 : {task_name}:{task_status},{x_pose},{y_pose}
        '''
        msg = request.allocation_lock

        # 如果只有 allocation_lock 收到訊息
        if msg == 'Create lock file':
            self.get_logger().info(msg)
            open(self.auctioneerlocker_file_path, 'w').close() 
        elif msg == 'Remove lock file':
            self.get_logger().info(msg)
            os.remove(self.auctioneerlocker_file_path)
        
        response.reply_lock = 'Done'
        
        return response
        


def main(args=None):
    rclpy.init(args=args)
    agent = AuctioneerLockerAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()