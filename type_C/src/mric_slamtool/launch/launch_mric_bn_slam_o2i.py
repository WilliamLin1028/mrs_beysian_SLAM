import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import time

'''
本程式用來啟動將 外部server(192.168.X.X)訊息 傳往 本地server(127.0.0.1) 的通溝程式
1. bilateral_topic_o2i.py - 用來將外部server發布的 topic (如:/{robot_name}_merge_map) 轉發成內部server要接收的topic
2. bilateral_receiver.py  - 改寫自 outside_receiver.py ,
'''


def generate_launch_description():
    ld = LaunchDescription()

    topic_o2i_agent = Node(
        package='mric_slamtool', 
        executable='bilateral_topic_o2i',  
    )

    mark_task_agent = Node(
        package='mric_slamtool', 
        executable='inside_mark_task',  
    )
    
    successrate_agent = Node(
        package='mric_slamtool', 
        executable='inside_successrate_agent',  
    )
    
    ld.add_action(topic_o2i_agent)
    ld.add_action(successrate_agent)
    ld.add_action(mark_task_agent)

    return ld
