import rclpy
import os

from rclpy.node import Node
# from rclpy.action import ActionServer
# from mric_action.action import RequestSuccessrate

from mric_slamtool_interfaces.srv import ConnectReciver
from ament_index_python.packages import get_package_share_directory


'''
此程式用來在 server 外回應其他機器人對此機器人執行當前 task array 中，未完成任務的成功率，邏輯如下:
1. 收到 request 先刪除此機器人所屬的成功率檔案({robot_name}_successrate.txt)
2. 建立 cmd_request_successrate.txt 的暫存檔案
3. 等待 {robot_name}_successrat.txt 的檔案存在
4. 讀取該檔案的字串({task_num}:{successrate};{task_num}:{successrate}...)並回傳
'''
robot_name = 'robot_D'    # 此機器人的名稱

class PubSuccessrateAgent(Node):
    def __init__(self):
        super().__init__(f'{robot_name}_SuccessRate_server')
        self.srv = self.create_service(ConnectReciver, f'{robot_name}_reciver', self.node_action)
        
        bringup_dir = get_package_share_directory('mric_slamtool')
        self.local_successrate_file_path = os.path.join(bringup_dir, 'database', 'local_successrate.txt')
        self.task_list_file_path = os.path.join(bringup_dir, 'database', 'task_list.txt')
        self.ongoing_task_file_path = os.path.join(bringup_dir, 'database', 'ongoing_task.txt')
        
        self.auctioneerlocker_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_wait_for_auctioneer.txt')
        self.cmd_request_sr_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_request_successrate.txt')
        self.cmd_update_ongoing_task_file_path = os.path.join(bringup_dir, 'cmd_folder', 'cmd_update_ongoing_task.txt')
        
        
    def check_file_existence(self, file_path):
        """
        檢查指定目錄內是否存在指定檔案
        參數:   directory (str): 要檢查的資料夾路徑
                filename (str): 要檢查的檔案名稱
        回傳:   str: 檔案是否存在的訊息
        """
        
        # 檢查檔案是否存在
        if os.path.isfile(file_path):
            return 'file is exist'
        else:
            return 'file is not exist'
    
    def read_ongoing_task(self, file_path):
        """
        從給定的字串中提取字串 , 格式: {task_name}:{status},{x_pose},{y_pose}
        """
        # 確認檔案是否已被開啟的假檔案
        lock_file = file_path + '.lock'
        if not os.path.exists(lock_file):
            open(lock_file, 'w').close()
            with open(file_path, 'r') as file:
                ongoing_task_string = file.read().strip()

            return ongoing_task_string
    
    def read_successrate_file(self, file_path):
        """
        字串格式: {task_name}:{successrate}
        """
        # 確認檔案是否已被開啟的假檔案
        lock_file = file_path + '.lock'
        if not os.path.exists(lock_file):
            open(lock_file, 'w').close()
            with open(file_path, 'r') as file:
                successrate_string = file.read().strip()
            
            # 刪除假檔案標示檔案已關閉
            if os.path.exists(lock_file):
                os.remove(lock_file)
            
            return successrate_string
    
    def node_action(self, request, response):
        '''
        service 的訊息 request 類型有三種,分別是
        "分配機鎖定與否的 allocation_lock",其內容為 : 
        "儲存 task list 並用於要求計算成功率的 tasks_list",其內容為 : {task_name}:{task_status},{x_pose},{y_pose};{task_name}:...
        "儲存新任務的 assigned_task",其內容為 : {task_name}:{task_status},{x_pose},{y_pose}
        '''
        a = request.allocation_lock
        b = request.tasks_list
        c = request.assigned_task
        
        if a != '' and b =='' and c == '':
            # 如果只有 allocation_lock 收到訊息
            if a == 'Create lock file':
                self.get_logger().info(a)
                open(self.auctioneerlocker_file_path, 'w').close() 
            elif a == 'Remove lock file':
                self.get_logger().info(a)
                os.remove(self.auctioneerlocker_file_path)
            
            response.reply_allocation = 'Done'
            response.reply_successrate = '' 
            response.reply_task = ''
        
        elif a == '' and b !='' and c == '':
            # 如果只有 tasks_list 收到訊息
            if os.path.exists(self.task_list_file_path):
                os.remove(self.task_list_file_path)
            
            if os.path.exists(self.local_file_path):
                os.remove(self.local_file_path)
            
            # 把所收到的 task list 存入 task_list.txt
            with open(self.task_list_file_path, 'w') as file:
                file.write(b)
            
            # 觸發 inside_successrate_agent.py 計算 local success rate
            open(self.cmd_request_sr_file_path, 'w').close()
            
            # 確認 local success rate 檔案 local_successrate.txt 是否存在
            count = 0
            while not os.path.exists(self.local_successrate_file_path):
                if count == 0:
                    self.get_logger().info('Wait for inside_successrate_agent.py calculate success rate.')
                    count +=1
                    
            os.remove(self.cmd_request_sr_file_path)
            
            # 當 inside_successrate_agent.py 計算完 local success rate 後, 讀取並用 service 回傳
            self.get_logger().info('Read local_successrate.txt.')
            successrate_string = self.read_successrate_file(self.local_successrate_file_path)
            
            response.reply_allocation = ''
            response.reply_successrate = successrate_string
            response.reply_task = ''
            
        elif a == '' and b =='' and c != '':
            # 如果機器人只收到 assigned_task 資訊, 會先讀取舊的 ongoing_task.txt 資料 , 並與
            # 新的 task 資訊比對 , 如果任務資訊不同 , 則把新的任務資訊覆蓋進 ongoing_task.txt 
            # , 並建立 cmd_update_ongoing_task.txt 
            ongoing_task_string = self.read_ongoing_task(self.ongoing_task_file_path)
            if ongoing_task_string != c:
                os.remove(self.ongoing_task_file_path)
                with open(self.ongoing_task_file_path, 'w') as file:
                    file.write(c)
                    
                open(self.cmd_update_ongoing_task_file_path, 'w').close()
            
            response.reply_allocation = ''
            response.reply_successrate = ''
            response.reply_task = 'Recive new task'
            
        else:
            self.get_logger().info('Status Error.')


def main(args=None):
    rclpy.init(args=args)
    agent = PubSuccessrateAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()