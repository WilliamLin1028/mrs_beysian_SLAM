import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import time

'''
程式架構 :
[database - ]       
                    [auctioneer]

                    [reply_successrate]
                    
                    [pub_map_workstatus]
                    
                    
                    NodeE ─────┐ 
        NodeA ───┐      ┌── NodeD ─── NodeF ─────┐
                NodeC ──┼───────────────────── NodeH
        NodeB ───┘      └── NodeG ───────────────┘
            NodeJ ── NodeK ───┘

'''
# 只有小寫的 m 和 c 能輸入
robot_type_1 = 'm'
robot_type_2 = 'M'

def generate_launch_description():
    ld = LaunchDescription()

    executor_agent = Node(
        package='mric_slamtool', 
        executable='inside_executor',  
    )

    save_robot_map_agent = Node(
        package='mric_slamtool',  
        executable='inside_save_robot_map',  
    )

    pub_merge_map_agent = Node(
        package='mric_slamtool', 
        executable='inside_pub_merge_map',  
    )

    mark_task_agent = Node(
        package='mric_slamtool', 
        executable='inside_mark_task',  
    )
    
    successrate_agent = Node(
        package='mric_slamtool', 
        executable='inside_successrate_agent',  
    )
    
    rb_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('smartrobot'),'launch',f'ominibot_{robot_type_1}lidar_launch.py'
                )])
    )
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('smartrobot'),'launch',f'mric_type{robot_type_2}_gmapping_navigation_launch.py'
                )])
    )
    

    #ld.add_action(rb_launch)
    #print(f'\nWaiting for ominibot_{robot_type_1}lidar_launch.py executing.')
    #time.sleep(15)
    ld.add_action(slam_launch)
    print(f'\nWaiting for mric_type{robot_type_2}_gmapping_navigation_launch.py executing.')
    time.sleep(15)
    # ld.add_action(save_robot_map_agent)
    # ld.add_action(pub_merge_map_agent)
    print(f'\n Executing self config agent')
    ld.add_action(executor_agent)
    ld.add_action(successrate_agent)
    ld.add_action(mark_task_agent)

    return ld
