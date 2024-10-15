import rclpy
import os

from ament_index_python.packages import get_package_share_directory

'''
此程式用來初始化 database 與 cmd_folder 的
'''

cmd_folder_file = ['cmd_start_experiment.txt',
                   'cmd_request_successrate.txt',
                   'cmd_robot_standby.txt',
                   'cmd_robot_ongoing.txt',
                   'cmd_robot_A_attend.txt',
                   'cmd_robot_B_attend.txt',
                   'cmd_robot_C_attend.txt',
                   'cmd_robot_E_attend.txt',
                   'cmd_robot_F_attend.txt',
                   'cmd_allocation.txt',
                   'cmd_update_ongoing_task.txt']

database_file = ['task_list.txt',
                 'ongoing_task.txt',
                 'local_successrate.txt',
                 'allocation_A_successrate.txt',
                 'allocation_B_successrate.txt',
                 'allocation_C_successrate.txt',
                 'allocation_D_successrate.txt',
                 'allocation_E_successrate.txt',
                 'send_A_new_task.txt',
                 'send_B_new_task.txt',
                 'send_C_new_task.txt',
                 'send_D_new_task.txt',
                 'send_E_new_task.txt']


def check_remove(file_path):
    if os.path.isfile(file_path):
        os.remove(file_path)

def initial_file(bringup_dir):
    bringup_dir = bringup_dir = get_package_share_directory('mric_slamtool')
    for file in cmd_folder_file:
        file_path = os.path.join(bringup_dir, 'cmd_folder', file)
        check_remove(file_path)
    
    for file in database_file:
        file_path = os.path.join(bringup_dir, 'database', file)
        check_remove(file_path)
    
        
if __name__ == '__main__':
    initial_file()